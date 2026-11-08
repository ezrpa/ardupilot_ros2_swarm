#!/usr/bin/env python3
"""Synthetic down-facing camera for running missions WITHOUT Gazebo.

Gazebo provides two things: physics (replaced by SITL's built-in model when
USE_GAZEBO is unset) and the camera images the mapping / SAR missions detect
weeds and the missing person in. This module replaces the second: it renders
a nadir frame from the drone's MEASURED pose and the same ground-truth files
the Gazebo worlds are generated from:

    weed_field/ground_truth.json      magenta weed discs (lat/lon/diameter)
    missing_person/ground_truth.json  hi-vis orange casualty (lat/lon/yaw)

The rendered frame then goes through the UNCHANGED CameraProcessor pipeline
(detect_color_blobs HSV bands, georeferencing, sidecar JSON, jpg), so the
prescription planner, SAR call logic and TAK bridge see exactly the data they
see in a Gazebo run. Rendering uses the exact inverse of the georeferencing
model (pinhole, nadir, body-fixed camera rotated by heading, flat ground), so
the only error left is pixel quantisation; there is no lens or GPS noise.

Selected with SIM_SENSORS=synthetic (set by start_tak_sim.sh); make_camera()
is the factory the drone nodes use. SIM_MODELS_DIR points at the models
directory (compose mounts src/custom_gz/models at /root/sim_models).
"""
import json
import math
import os

from swarm_control.perception import camera_processor as cp
from swarm_control.core import config, geo

DEFAULT_MODELS_DIR = "/root/sim_models"
FRAME_W, FRAME_H = config.CAMERA_WIDTH_PX, config.CAMERA_HEIGHT_PX
METERS_PER_DEG_LAT = geo.M_PER_DEG_LAT  # georef and projection share geo

# Colours are BGR. Each must fall inside its detector's HSV band
# (camera_processor.WEED_HSV_* / PERSON_HSV_*) and the background must not.
WEED_BGR = (255, 0, 255)        # magenta -> OpenCV H 150
PERSON_BGR = (0, 100, 255)      # hi-vis orange -> OpenCV H ~12, S 255
BOOT_BGR = (30, 30, 30)
SOIL_BGR = (60, 105, 120)       # H ~19, low S: outside both bands
CROP_BGR = (50, 120, 55)        # green rows, H ~62
PERSON_LEN_M, PERSON_WID_M = 1.9, 0.45   # missing_person model footprint
SKIN_BGR = (120, 160, 205)
# Decoys are deliberately inside the person colour band (hi-vis orange /
# orange-red): only their SHAPE can tell them apart.
DECOY_BGR = {"cone": (0, 95, 250), "tarp": (10, 110, 245),
             "kayak": (0, 70, 235), "backpack": (0, 105, 240)}


def _load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def load_scenario(models_dir=None, scenario=None):
    """(weeds, person, distractors). scenario (env SIM_SCENARIO, e.g.
    'sar_scenario') names a models/<scenario>/ground_truth.json whose person
    and distractors replace the Gazebo missing_person; the Gazebo worlds are
    untouched."""
    models_dir = models_dir or os.environ.get("SIM_MODELS_DIR",
                                              DEFAULT_MODELS_DIR)
    weeds, person = load_ground_truth(models_dir)
    distractors = []
    scenario = scenario if scenario is not None else \
        os.environ.get("SIM_SCENARIO", "")
    if scenario:
        sc = _load_json(os.path.join(models_dir, scenario,
                                     "ground_truth.json")) or {}
        person = sc.get("person", person)
        distractors = sc.get("distractors", [])
    return weeds, person, distractors


def load_ground_truth(models_dir=None):
    """Returns (weeds, person). weeds: [{'lat','lon','diameter'}]; person:
    {'lat','lon','yaw'} or None. Missing files give empty results."""
    models_dir = models_dir or os.environ.get("SIM_MODELS_DIR",
                                              DEFAULT_MODELS_DIR)
    w = _load_json(os.path.join(models_dir, "weed_field", "ground_truth.json"))
    p = _load_json(os.path.join(models_dir, "missing_person",
                                "ground_truth.json"))
    weeds = [{"lat": d["lat"], "lon": d["lon"],
              "diameter": float(d.get("diameter", 1.5))}
             for d in (w or {}).get("weeds", [])]
    person = (p or {}).get("person")
    return weeds, person


def project(lat0, lon0, alt, heading, lat, lon, w=FRAME_W, h=FRAME_H):
    """Ground point (lat, lon) -> pixel (x, y) for a nadir camera at
    (lat0, lon0, alt AGL) with compass heading (rad). Exact inverse of the
    georeferencing in camera_processor.detect_color_blobs."""
    f_px = (w / 2.0) / math.tan(cp.CAMERA_HFOV_RAD / 2.0)
    north, east = geo.ne_between(lat0, lon0, lat, lon)
    psi = heading or 0.0
    forward = north * math.cos(psi) + east * math.sin(psi)
    right = -north * math.sin(psi) + east * math.cos(psi)
    return (w / 2.0 + right / alt * f_px, h / 2.0 - forward / alt * f_px)


def _ne_of(lat0, lon0, lat, lon):
    return geo.ne_between(lat0, lon0, lat, lon)


def _rect_ne(cn, ce, length, width, yaw, along=0.0, across=0.0):
    """Corners (north, east) of a length x width box whose long axis points
    along ENU yaw (0 = east, CCW), offset along/across that axis."""
    ux, uy = math.cos(yaw), math.sin(yaw)        # east, north of long axis
    vx, vy = -uy, ux
    cx = ce + ux * along + vx * across
    cy = cn + uy * along + vy * across
    hl, hw = length / 2.0, width / 2.0
    return [(cy + uy * a + vy * b, cx + ux * a + vx * b)
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def render_frame(lat0, lon0, alt, heading, weeds=(), person=None,
                 w=FRAME_W, h=FRAME_H, distractors=(), realism=False,
                 rng=None):
    """Render a BGR nadir frame (numpy array), or None without cv2 / a
    usable pose.

    realism=True adds what makes detection non-trivial: blotchy ground
    texture, ground-anchored tree shadows, an articulated casualty (torso,
    legs, arms, head, boots), orange decoys (`distractors`), exposure
    changes, optics blur and sensor noise."""
    if not cp._CV2_OK or lat0 is None or lon0 is None or not alt or alt <= 0.5:
        return None
    import numpy as np
    cv2 = cp.cv2
    rng = rng or np.random.default_rng()
    f_px = (w / 2.0) / math.tan(cp.CAMERA_HFOV_RAD / 2.0)
    m2px = f_px / alt
    psi = heading or 0.0

    def px(n, e):     # metres from the drone (north, east) -> pixel
        fwd = n * math.cos(psi) + e * math.sin(psi)
        rgt = -n * math.sin(psi) + e * math.cos(psi)
        return (w / 2.0 + rgt * m2px, h / 2.0 - fwd * m2px)

    def poly(corners_ne, colour, img):
        pts = np.array([px(n, e) for n, e in corners_ne], np.float32)
        cv2.fillPoly(img, [np.round(pts).astype(np.int32)], colour,
                     lineType=cv2.LINE_AA if realism else cv2.LINE_8)

    frame = np.empty((h, w, 3), np.uint8)
    frame[:] = SOIL_BGR
    if realism:   # low-frequency soil / grass blotches
        small = rng.normal(0.0, 1.0, (6, 8, 1)).astype(np.float32)
        blot = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)
        tint = np.array([[-6.0, 10.0, 4.0]], np.float32)   # toward grass
        frame = np.clip(frame.astype(np.float32)
                        + blot[..., None] * tint * 2.0, 0, 255).astype(np.uint8)

    # Crop rows running north-south, anchored to the ground.
    row_pitch_m = 0.75
    for k in range(-60, 61):
        e = k * row_pitch_m
        a = px(60.0, e)
        b = px(-60.0, e)
        cv2.line(frame, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                 CROP_BGR, max(1, int(0.35 * m2px)))

    if realism:   # tree shadows: hashed 15 m world grid, stable per place
        shade = np.ones((h, w), np.float32)
        n0, e0 = _ne_of(40.0, -105.0, lat0, lon0)          # absolute-ish
        cell = 15.0
        reach = alt * 1.2
        for i in range(int((n0 - reach) // cell), int((n0 + reach) // cell) + 1):
            for j in range(int((e0 - reach) // cell), int((e0 + reach) // cell) + 1):
                hsh = (i * 73856093 ^ j * 19349663) & 0xFFFF
                if hsh % 5:
                    continue
                tn = i * cell + (hsh % 97) / 97.0 * cell - n0
                te = j * cell + (hsh % 89) / 89.0 * cell - e0
                cx, cy = px(tn, te)
                r = (2.0 + (hsh % 7) * 0.4) * m2px
                cv2.ellipse(shade, (int(cx), int(cy)), (int(r * 1.4), int(r)),
                            math.degrees(-psi) + 30, 0, 360, 0.55, -1,
                            cv2.LINE_AA)
        shade = cv2.GaussianBlur(shade, (0, 0), max(1.0, 0.6 * m2px))
        frame = (frame.astype(np.float32) * shade[..., None]).astype(np.uint8)

    for wd in weeds:
        x, y = project(lat0, lon0, alt, psi, wd["lat"], wd["lon"], w, h)
        r = max(1, int(round(wd["diameter"] / 2.0 * m2px)))
        if -r <= x <= w + r and -r <= y <= h + r:
            cv2.circle(frame, (int(round(x)), int(round(y))), r, WEED_BGR, -1)

    for d in distractors:
        dn, de = _ne_of(lat0, lon0, d["lat"], d["lon"])
        if abs(dn) > alt * 1.5 or abs(de) > alt * 1.5:
            continue
        kind, colour = d.get("kind"), DECOY_BGR.get(d.get("kind"), PERSON_BGR)
        if kind == "cone":
            cx, cy = px(dn, de)
            r = max(1, int(round(d["length_m"] / 2.0 * m2px)))
            cv2.circle(frame, (int(cx), int(cy)), r, colour, -1, cv2.LINE_AA)
            cv2.circle(frame, (int(cx), int(cy)), max(1, r // 2),
                       (235, 235, 235), max(1, r // 4), cv2.LINE_AA)
        else:
            poly(_rect_ne(dn, de, d["length_m"], d["width_m"], d["yaw"]),
                 colour, frame)

    if person:
        pn, pe = _ne_of(lat0, lon0, person["lat"], person["lon"])
        yaw = float(person.get("yaw", 0.0))
        if realism:   # articulated casualty (missing_person model parts)
            for along, across, L, W in ((0.30, 0.0, 0.62, 0.42),     # torso
                                        (-0.45, 0.0, 0.88, 0.34),    # legs
                                        (0.30, 0.29, 0.58, 0.12),    # arms
                                        (0.30, -0.29, 0.58, 0.12)):
                poly(_rect_ne(pn, pe, L, W, yaw, along, across),
                     PERSON_BGR, frame)
            hx, hy = px(*_rect_ne(pn, pe, 0.0, 0.0, yaw, 0.75)[0])
            cv2.circle(frame, (int(hx), int(hy)), max(1, int(0.11 * m2px)),
                       SKIN_BGR, -1, cv2.LINE_AA)
            poly(_rect_ne(pn, pe, 0.14, 0.30, yaw, -0.95), BOOT_BGR, frame)
        else:
            poly(_rect_ne(pn, pe, PERSON_LEN_M, PERSON_WID_M, yaw),
                 PERSON_BGR, frame)
            bx, by = px(*_rect_ne(pn, pe, 0.0, 0.0, yaw,
                                  -PERSON_LEN_M / 2.0 * 1.05)[0])
            cv2.circle(frame, (int(round(bx)), int(round(by))),
                       max(1, int(0.12 * m2px)), BOOT_BGR, -1)

    if realism:   # exposure, optics, sensor
        gain = rng.uniform(0.85, 1.12)
        f = frame.astype(np.float32) * gain
        f = cv2.GaussianBlur(f, (0, 0), 0.8)
        f += rng.normal(0.0, 5.0, f.shape).astype(np.float32)
        frame = np.clip(f, 0, 255).astype(np.uint8)
    return frame


class SyntheticCamera(cp.CameraProcessor):
    """CameraProcessor whose frames are rendered from ground truth instead
    of decoded from a Gazebo RTP stream. Same public interface: capture(),
    last_meta, active, close()."""

    def __init__(self, drone_id, udp_port=0, out_dir="/root/logs",
                 logger=None, detectors=None, models_dir=None):
        # udp_port=0 makes the base class skip opening a stream; logger is
        # attached afterwards so its "camera disabled" notice isn't logged.
        super().__init__(drone_id, 0, out_dir=out_dir, logger=None,
                         detectors=detectors)
        self._log = logger
        self.weeds, self.person, self.distractors = load_scenario(models_dir)
        self.realism = os.environ.get("SIM_REALISM", "1") != "0"
        self._synthetic_ok = cp._CV2_OK
        if self._synthetic_ok:
            os.makedirs(self.out_dir, exist_ok=True)
            self._info(f"synthetic camera: {len(self.weeds)} weeds, "
                       f"person={'yes' if self.person else 'no'}, "
                       f"{len(self.distractors)} decoys, "
                       f"realism={'on' if self.realism else 'off'}")

    @property
    def active(self):
        return self._synthetic_ok

    def _grab_frame(self, lat=None, lon=None, alt=None, heading=None):
        return render_frame(lat, lon, alt, heading, self.weeds, self.person,
                            distractors=self.distractors,
                            realism=self.realism)

    def close(self):
        self._synthetic_ok = False


def make_camera(drone_id, udp_port, logger=None, detectors=None,
                out_dir="/root/logs"):
    """Camera factory for the drone nodes: SIM_SENSORS=synthetic renders
    frames from ground truth (no Gazebo); anything else uses the Gazebo
    GstCameraPlugin stream on udp_port."""
    if os.environ.get("SIM_SENSORS", "").strip().lower() == "synthetic":
        return SyntheticCamera(drone_id, out_dir=out_dir, logger=logger,
                               detectors=detectors)
    return cp.CameraProcessor(drone_id, udp_port, out_dir=out_dir,
                              logger=logger, detectors=detectors)

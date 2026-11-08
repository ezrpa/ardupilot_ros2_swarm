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

try:
    from scripts import camera_processor as cp
except ImportError:  # host-side tests run with scripts/ on sys.path
    import camera_processor as cp

DEFAULT_MODELS_DIR = "/root/sim_models"
FRAME_W, FRAME_H = 640, 480     # base_drone_cam/model.sdf image size
METERS_PER_DEG_LAT = 111111.0   # same constant detect_color_blobs inverts

# Colours are BGR. Each must fall inside its detector's HSV band
# (camera_processor.WEED_HSV_* / PERSON_HSV_*) and the background must not.
WEED_BGR = (255, 0, 255)        # magenta -> OpenCV H 150
PERSON_BGR = (0, 100, 255)      # hi-vis orange -> OpenCV H ~12, S 255
BOOT_BGR = (30, 30, 30)
SOIL_BGR = (60, 105, 120)       # H ~19, low S: outside both bands
CROP_BGR = (50, 120, 55)        # green rows, H ~62
PERSON_LEN_M, PERSON_WID_M = 1.9, 0.45   # missing_person model footprint


def _load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


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
    north = (lat - lat0) * METERS_PER_DEG_LAT
    east = (lon - lon0) * METERS_PER_DEG_LAT * math.cos(math.radians(lat0))
    psi = heading or 0.0
    forward = north * math.cos(psi) + east * math.sin(psi)
    right = -north * math.sin(psi) + east * math.cos(psi)
    return (w / 2.0 + right / alt * f_px, h / 2.0 - forward / alt * f_px)


def render_frame(lat0, lon0, alt, heading, weeds=(), person=None,
                 w=FRAME_W, h=FRAME_H):
    """Render a BGR nadir frame (numpy array), or None without cv2 / a
    usable pose."""
    if not cp._CV2_OK or lat0 is None or lon0 is None or not alt or alt <= 0.5:
        return None
    import numpy as np
    cv2 = cp.cv2
    f_px = (w / 2.0) / math.tan(cp.CAMERA_HFOV_RAD / 2.0)
    m2px = f_px / alt
    frame = np.empty((h, w, 3), np.uint8)
    frame[:] = SOIL_BGR

    # Crop rows running north-south, anchored to the ground (they rotate and
    # scroll with the drone like real rows) so the stitched map looks right.
    row_pitch_m = 0.75
    psi = heading or 0.0
    for k in range(-60, 61):
        e = k * row_pitch_m
        a = project(lat0, lon0, alt, psi,
                    lat0 + 60 / METERS_PER_DEG_LAT,
                    lon0 + e / (METERS_PER_DEG_LAT
                                * math.cos(math.radians(lat0))), w, h)
        b = project(lat0, lon0, alt, psi,
                    lat0 - 60 / METERS_PER_DEG_LAT,
                    lon0 + e / (METERS_PER_DEG_LAT
                                * math.cos(math.radians(lat0))), w, h)
        cv2.line(frame, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                 CROP_BGR, max(1, int(0.35 * m2px)))

    for wd in weeds:
        x, y = project(lat0, lon0, alt, psi, wd["lat"], wd["lon"], w, h)
        r = max(1, int(round(wd["diameter"] / 2.0 * m2px)))
        if -r <= x <= w + r and -r <= y <= h + r:
            cv2.circle(frame, (int(round(x)), int(round(y))), r, WEED_BGR, -1)

    if person:
        x, y = project(lat0, lon0, alt, psi, person["lat"], person["lon"],
                       w, h)
        # Long axis direction: world yaw is ENU (0 = east, CCW). Express it
        # in body frame, then in image axes (x = right, y = -forward).
        yaw = float(person.get("yaw", 0.0))
        n_dir, e_dir = math.sin(yaw), math.cos(yaw)
        fwd = n_dir * math.cos(psi) + e_dir * math.sin(psi)
        rgt = -n_dir * math.sin(psi) + e_dir * math.cos(psi)
        ux, uy = rgt, -fwd                    # unit vector along the body
        vx, vy = -uy, ux                      # across the body
        hl = PERSON_LEN_M / 2.0 * m2px
        hw = PERSON_WID_M / 2.0 * m2px
        pts = np.array([[x + ux * hl + vx * hw, y + uy * hl + vy * hw],
                        [x + ux * hl - vx * hw, y + uy * hl - vy * hw],
                        [x - ux * hl - vx * hw, y - uy * hl - vy * hw],
                        [x - ux * hl + vx * hw, y - uy * hl + vy * hw]],
                       np.float32)
        cv2.fillPoly(frame, [np.round(pts).astype(np.int32)], PERSON_BGR)
        boot = (int(round(x - ux * hl * 1.05)), int(round(y - uy * hl * 1.05)))
        cv2.circle(frame, boot, max(1, int(0.12 * m2px)), BOOT_BGR, -1)
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
        self.weeds, self.person = load_ground_truth(models_dir)
        self._synthetic_ok = cp._CV2_OK
        if self._synthetic_ok:
            os.makedirs(self.out_dir, exist_ok=True)
            self._info(f"synthetic camera: {len(self.weeds)} weeds, "
                       f"person={'yes' if self.person else 'no'}")

    @property
    def active(self):
        return self._synthetic_ok

    def _grab_frame(self, lat=None, lon=None, alt=None, heading=None):
        return render_frame(lat, lon, alt, heading, self.weeds, self.person)

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

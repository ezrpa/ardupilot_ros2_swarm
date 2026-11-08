#!/usr/bin/env python3
"""OpenCV capture pipeline for the drone's down-facing Gazebo camera.

Each drone model streams H.264/RTP over UDP via GstCameraPlugin on port
5600 + instance (see src/custom_gz/models/base_drone*/model.sdf). This module
decodes that stream with OpenCV's GStreamer backend and provides
waypoint-triggered, geotagged frame capture plus ArUco marker detection and
colour-blob detection (magenta weeds; hi-vis orange person for SAR).

Design constraints:
 - MUST be non-fatal: a missing camera, cv2, or GStreamer support must never
   break the flight mission. Every public method degrades to a no-op.
 - Frames are saved under /root/logs (docker-mounted to ./logs on the host) as
   logs/drone_<id>/wp_<n>.jpg plus a JSON sidecar with the geotag, so the
   host-side mosaic script can place them on the field map.
"""
import json
import math
import os
import threading
import time

try:
    import geo
except ImportError:  # installed as the `scripts` python package
    from scripts import geo

try:
    import cv2
    _CV2_OK = True
except ImportError:  # pragma: no cover - depends on container image
    cv2 = None
    _CV2_OK = False

# Weed detection: the sim's weeds (model weed_field) are pure magenta blobs,
# so a tight HSV band + area filter is a reliable detector. Camera intrinsics
# from base_drone_cam/model.sdf, used for nadir georeferencing.
CAMERA_HFOV_RAD = 1.2
WEED_HSV_LOW = (140, 100, 80)
WEED_HSV_HIGH = (170, 255, 255)
WEED_MIN_AREA_PX = 50

# Search & rescue: the missing person (model missing_person) wears a hi-vis
# orange coverall. Orange sits at OpenCV H ~10; the band is kept below the
# farm textures' soil/wheat/sunflower hues (H 15-24) and demands high
# saturation, which those textures lack. A 1.5 x 0.45 m figure is ~160 px at
# 30 m AGL (6.4 cm/px), so 40 px still rejects speckle.
PERSON_HSV_LOW = (0, 180, 120)
PERSON_HSV_HIGH = (14, 255, 255)
PERSON_MIN_AREA_PX = 40

# name -> (hsv_low, hsv_high, min_area_px). Each lands in the sidecar as
# "<name>_detections".
DEFAULT_DETECTORS = {"weed": (WEED_HSV_LOW, WEED_HSV_HIGH, WEED_MIN_AREA_PX)}
PERSON_DETECTORS = {"person": (PERSON_HSV_LOW, PERSON_HSV_HIGH,
                               PERSON_MIN_AREA_PX)}

# Shape-verified person detection (SAR). Colour alone also fires on orange
# cones, tarps, kayaks and backpacks, so each orange blob is measured in
# METRES (ground sample distance = alt / focal_px) and scored against a
# person lying on the ground: ~1.3-2.0 m long, ~0.4-1.0 m wide (arms),
# compact (solidity). The colour band is wider than PERSON_HSV_* to survive
# sensor noise, shadow and exposure changes; shape does the rejecting.
PERSON_SHAPE_HSV_LOW = (0, 140, 90)
PERSON_SHAPE_HSV_HIGH = (16, 255, 255)
PERSON_LEN_M = (1.6, 0.35)           # mean, sigma (arms + blur widen it)
PERSON_WID_M = (0.75, 0.25)
PERSON_MIN_CONFIDENCE = 0.35


def detect_color_blobs(frame, hsv_low, hsv_high, min_area_px,
                       lat=None, lon=None, alt=None, heading=None):
    """Find blobs inside an HSV band; returns [{'centroid_px', 'bbox',
    'area_px', 'lat', 'lon'}]. lat/lon are georeferenced from the drone pose
    with a nadir camera model, or None when the pose is unknown. Never raises.

    The camera is body-fixed (base_drone_cam mounts it at pitch 90 with no
    yaw offset), so image-right is the vehicle's right and image-up is its
    nose. Pixel offsets are therefore BODY-frame and must be rotated by the
    compass heading (radians) to become north/east; skipping that rotation put
    detections up to a full 2x their nadir offset away, since ArduPilot yaws
    toward each waypoint and reverses heading on alternating serpentine legs.
    heading=None falls back to the old north-aligned assumption."""
    if not _CV2_OK:
        return []
    try:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_low, hsv_high)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = frame.shape[:2]
        f_px = (w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area_px:
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            cx = bx + bw / 2.0
            cy = by + bh / 2.0
            det = {
                "centroid_px": [cx, cy],
                "bbox": [bx, by, bw, bh],
                "area_px": area,
                "lat": None,
                "lon": None,
            }
            if lat is not None and lon is not None and alt is not None:
                right = (cx - w / 2.0) / f_px * alt
                forward = -(cy - h / 2.0) / f_px * alt
                psi = 0.0 if heading is None else heading
                north = forward * math.cos(psi) - right * math.sin(psi)
                east = forward * math.sin(psi) + right * math.cos(psi)
                det["lat"], det["lon"] = geo.offset_latlon(lat, lon,
                                                           north, east)
            detections.append(det)
        return detections
    except Exception:
        return []


def _gauss(x, mu, sigma):
    return math.exp(-0.5 * ((x - mu) / sigma) ** 2)


def detect_person(frame, lat=None, lon=None, alt=None, heading=None,
                  min_confidence=PERSON_MIN_CONFIDENCE, rejected=None):
    """Shape-verified person detector. Returns detections like
    detect_color_blobs plus 'confidence', 'length_m', 'width_m'. Candidates
    that fail are appended to `rejected` (if a list) with a 'reason', so a
    run can show WHY an orange object was not called a person. Never
    raises; needs alt (AGL) to measure sizes."""
    if not _CV2_OK or not alt or alt <= 0:
        return []
    try:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, PERSON_SHAPE_HSV_LOW, PERSON_SHAPE_HSV_HIGH)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(
                                    cv2.MORPH_ELLIPSE, (7, 7)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        f_px = (w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
        gsd = alt / f_px                       # metres per pixel
        min_px = max(8.0, (0.15 / gsd) ** 2)   # ignore specks < 0.15 m
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_px:
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            length_m, width_m = max(rw, rh) * gsd, min(rw, rh) * gsd
            hull = cv2.contourArea(cv2.convexHull(c)) or area
            solidity = area / hull
            aspect = length_m / max(width_m, 1e-3)
            conf = (_gauss(length_m, *PERSON_LEN_M)
                    * _gauss(width_m, *PERSON_WID_M)
                    * min(1.0, solidity / 0.75))
            bx, by, bw, bh = cv2.boundingRect(c)
            cx, cy = bx + bw / 2.0, by + bh / 2.0
            det = {"centroid_px": [cx, cy], "bbox": [bx, by, bw, bh],
                   "area_px": area, "lat": None, "lon": None,
                   "confidence": round(conf, 3),
                   "length_m": round(length_m, 2),
                   "width_m": round(width_m, 2),
                   "solidity": round(solidity, 2)}
            if lat is not None and lon is not None:
                right = (cx - w / 2.0) * gsd
                forward = -(cy - h / 2.0) * gsd
                psi = 0.0 if heading is None else heading
                north = forward * math.cos(psi) - right * math.sin(psi)
                east = forward * math.sin(psi) + right * math.cos(psi)
                det["lat"], det["lon"] = geo.offset_latlon(lat, lon,
                                                           north, east)
            # A blob cut by the frame edge can't be measured: a clipped
            # kayak or tarp can look exactly person-sized. Next frame (or
            # the confirm pass, which centres the target) will see it whole.
            clipped = bx <= 1 or by <= 1 or bx + bw >= w - 1 or by + bh >= h - 1
            if conf >= min_confidence and not clipped:
                out.append(det)
            elif rejected is not None:
                if clipped:
                    reason = "cut off by frame edge"
                elif length_m < 0.9:
                    reason = f"too small ({length_m:.1f} m)"
                elif length_m > 2.6 or width_m > 1.2:
                    reason = f"too large ({length_m:.1f} x {width_m:.1f} m)"
                elif aspect < 1.8:
                    reason = f"not elongated ({length_m:.1f} x {width_m:.1f} m)"
                else:
                    reason = f"shape mismatch ({length_m:.1f} x {width_m:.1f} m)"
                rejected.append(dict(det, reason=reason))
        out.sort(key=lambda d: -d["confidence"])
        return out
    except Exception:
        return []


# Detector map entries may be a callable (frame, lat, lon, alt, heading,
# rejected=list) instead of an HSV tuple.
PERSON_SHAPE_DETECTORS = {"person": detect_person}


class CameraProcessor:
    """Grabs frames from a GstCameraPlugin UDP stream and geotags them."""

    def __init__(self, drone_id, udp_port, out_dir="/root/logs", logger=None,
                 detectors=None):
        self.drone_id = drone_id
        self.detectors = dict(DEFAULT_DETECTORS if detectors is None
                              else detectors)
        self.last_meta = None  # sidecar dict of the most recent capture
        self.udp_port = udp_port
        self.out_dir = os.path.join(out_dir, f"drone_{drone_id}")
        self._log = logger
        self._cap = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._reader = None
        self._running = False

        if not _CV2_OK:
            self._warn("cv2 not available - camera capture disabled")
            return
        if not udp_port:
            self._info("no camera_port configured - camera capture disabled")
            return

        os.makedirs(self.out_dir, exist_ok=True)
        self._open_stream()

    # ------------------------------------------------------------------ util
    def _info(self, msg):
        if self._log:
            self._log.info(f"[camera] {msg}")

    def _warn(self, msg):
        if self._log:
            self._log.warning(f"[camera] {msg}")

    @property
    def active(self):
        return self._cap is not None and self._running

    # ---------------------------------------------------------------- stream
    def _open_stream(self):
        """Open the RTP/H.264 stream via OpenCV's GStreamer backend."""
        pipeline = (
            f"udpsrc port={self.udp_port} "
            "caps=\"application/x-rtp, media=video, encoding-name=H264\" "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert ! video/x-raw, format=BGR "
            "! appsink drop=true max-buffers=1 sync=false"
        )
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        except Exception as e:  # GStreamer backend missing, etc.
            self._warn(f"failed to open stream on udp:{self.udp_port}: {e}")
            return
        if not cap.isOpened():
            self._warn(
                f"could not open GStreamer pipeline on udp:{self.udp_port} "
                "(stream not enabled yet, or cv2 built without GStreamer)")
            return

        self._cap = cap
        self._running = True
        # Dedicated reader thread keeps the appsink drained so capture()
        # always returns the freshest frame instead of a stale buffered one.
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._info(f"camera stream open on udp:{self.udp_port}")

    def _read_loop(self):
        while self._running:
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.05)

    # --------------------------------------------------------------- capture
    def _grab_frame(self, lat=None, lon=None, alt=None, heading=None):
        """Freshest stream frame (a copy), or None. The pose arguments are
        unused here; synthetic_camera.SyntheticCamera renders from them."""
        with self._lock:
            return (None if self._latest_frame is None
                    else self._latest_frame.copy())

    def capture(self, wp_index, lat=None, lon=None, alt=None, extra=None,
                prefix="wp", heading=None):
        """Save the freshest frame as <prefix>_<n>.jpg with a JSON geotag
        sidecar (ArUco + one "<name>_detections" list per configured
        detector). heading (radians) rotates body-frame pixel offsets to
        north/east. Returns the image path, or None if no frame was available;
        the sidecar dict is also kept in self.last_meta.
        """
        self.last_meta = None
        if not self.active:
            return None

        frame = self._grab_frame(lat=lat, lon=lon, alt=alt, heading=heading)
        if frame is None:
            self._warn(f"{prefix} {wp_index}: no frame received yet")
            return None

        detections = self._detect_aruco(frame)
        found, rejected = {}, {}
        for name, spec in self.detectors.items():
            if callable(spec):
                rejected[name] = []
                found[name] = spec(frame, lat=lat, lon=lon, alt=alt,
                                   heading=heading, rejected=rejected[name])
            else:
                lo, hi, min_area = spec
                found[name] = detect_color_blobs(frame, lo, hi, min_area,
                                                 lat=lat, lon=lon, alt=alt,
                                                 heading=heading)

        stem = f"{prefix}_{wp_index:03d}"
        img_path = os.path.join(self.out_dir, f"{stem}.jpg")
        cv2.imwrite(img_path, frame)

        meta = {
            "drone_id": self.drone_id,
            "wp_index": wp_index,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            # Recorded so a run can be audited after the fact: heading is what
            # turns body-frame pixel offsets into north/east.
            "heading_deg": (None if heading is None
                            else math.degrees(heading) % 360.0),
            "unix_time": time.time(),
            "aruco_detections": detections,
        }
        for name, dets in found.items():
            meta[f"{name}_detections"] = dets
        for name, rej in rejected.items():
            meta[f"{name}_rejected"] = rej
        if extra:
            meta.update(extra)
        with open(os.path.join(self.out_dir, f"{stem}.json"), "w") as f:
            json.dump(meta, f, indent=2)
        self.last_meta = meta

        self._info(
            f"{stem}: saved {os.path.basename(img_path)}"
            + (f" ({len(detections)} aruco)" if detections else "")
            + "".join(f" ({len(d)} {n})" for n, d in found.items() if d)
            + "".join(f" ({len(r)} {n} candidates rejected: "
                      f"{', '.join(x['reason'] for x in r)})"
                      for n, r in rejected.items() if r))
        return img_path

    def _detect_aruco(self, frame):
        """Detect ArUco markers (DICT_4X4_50); returns [{'id', 'corners'}]."""
        try:
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            try:  # OpenCV >= 4.7 API
                detector = aruco.ArucoDetector(dictionary)
                corners, ids, _ = detector.detectMarkers(frame)
            except AttributeError:  # OpenCV 4.5.x API (Ubuntu 22.04)
                corners, ids, _ = aruco.detectMarkers(frame, dictionary)
        except Exception:
            return []
        if ids is None:
            return []
        return [
            {"id": int(i[0]), "corners": c.reshape(-1, 2).tolist()}
            for i, c in zip(ids, corners)
        ]

    def _detect_weeds(self, frame, lat=None, lon=None, alt=None, heading=None):
        """Detect magenta weed blobs (see detect_color_blobs)."""
        return detect_color_blobs(frame, WEED_HSV_LOW, WEED_HSV_HIGH,
                                  WEED_MIN_AREA_PX, lat=lat, lon=lon, alt=alt,
                                  heading=heading)

    def _detect_person(self, frame, lat=None, lon=None, alt=None,
                       heading=None):
        """Detect the hi-vis orange missing person (see detect_color_blobs)."""
        return detect_color_blobs(frame, PERSON_HSV_LOW, PERSON_HSV_HIGH,
                                  PERSON_MIN_AREA_PX, lat=lat, lon=lon,
                                  alt=alt, heading=heading)

    # ---------------------------------------------------------------- close
    def close(self):
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

"""Single-frame mosquito-candidate detection: background subtraction against a
static reference frame, restricted to a ROI, blob-area filtered.

Same algorithm and parameter names as
``test_videos_particle_tracking/src/particle_tracking/detection.py`` (diff
from a static background frame, threshold, ROI mask, morphological
open/close, contour candidates) -- ported here so mosquito_preference_assay
stays a self-contained ROS package. What's dropped: that pipeline's
frame-to-frame nearest-neighbor linking and static-pixel pre-pass, which need
a whole recorded sequence up front. A live trigger only needs "is there a
mosquito-sized blob in the ROI right now" -- consecutive-frame debouncing in
mosquito_detector_node.py stands in for that.
"""

import cv2
import numpy as np


def find_candidates(gray, background, *, diff_threshold, min_area, max_area,
                     morph_kernel, roi=None):
    """Return every foreground blob with area in [min_area, max_area], largest
    first, as dicts {cx, cy, area, bbox} (bbox = (x, y, w, h)).

    gray, background: single-channel (grayscale) images, same shape.
    roi: optional [x0, y0, x1, y1] pixel box (x1/y1 exclusive, full-frame
    coords); pixels outside it are ignored.
    """
    if gray.shape != background.shape:
        raise ValueError(f"frame shape {gray.shape} != background shape {background.shape}")

    diff = cv2.absdiff(gray, background)
    _, mask = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)

    if roi is not None:
        x0, y0, x1, y1 = roi
        roi_mask = np.zeros_like(mask)
        roi_mask[y0:y1, x0:x1] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

    if morph_kernel > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or (max_area is not None and area > max_area):
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        candidates.append({
            "cx": m["m10"] / m["m00"],
            "cy": m["m01"] / m["m00"],
            "area": area,
            "bbox": cv2.boundingRect(c),
        })

    candidates.sort(key=lambda c: c["area"], reverse=True)
    return candidates


def parse_roi(text):
    """Parse a "x0,y0,x1,y1" pixel-box string (x1/y1 exclusive) into a tuple
    of 4 ints, or None for an empty string (whole frame)."""
    text = (text or "").strip()
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 4:
        raise ValueError(f"roi {text!r} must be 'x0,y0,x1,y1'")
    return tuple(int(float(p)) for p in parts)

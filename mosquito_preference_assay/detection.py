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


def find_candidates(
    gray, background, *, diff_threshold, min_area, max_area, morph_kernel, roi=None,
):
    """Return every foreground blob with area in [min_area, max_area], largest
    first, as dicts {cx, cy, area, bbox} (bbox = (x, y, w, h)).

    gray, background: single-channel (grayscale) images, same shape.
    roi: optional [x0, y0, x1, y1] pixel box (x1/y1 exclusive, full-frame
    coords); pixels outside it are ignored.
    """
    if gray.shape != background.shape:
        raise ValueError(f"frame shape {gray.shape} != background shape {background.shape}")

    # Crop to the ROI FIRST, so the diff/threshold/morphology only ever touch
    # ROI pixels -- doing them full-frame and masking afterwards costs ~6.5x
    # more per frame (7.8 ms vs 1.2 ms on 1440x1080 with the real arena ROI),
    # which is what caps the achievable frame rate. Same approach as the
    # reference pipeline in test_videos_particle_tracking. Centroids and
    # bounding boxes are offset back to full-frame coordinates below, so the
    # returned values are unchanged (verified identical on real footage).
    x_offset, y_offset = 0, 0
    if roi is not None:
        x0, y0, x1, y1 = roi
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(gray.shape[1], x1), min(gray.shape[0], y1)
        gray = gray[y0:y1, x0:x1]
        background = background[y0:y1, x0:x1]
        x_offset, y_offset = x0, y0

    diff = cv2.absdiff(gray, background)
    _, mask = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)

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
        x, y, w, h = cv2.boundingRect(c)
        candidates.append({
            "cx": x_offset + m["m10"] / m["m00"],
            "cy": y_offset + m["m01"] / m["m00"],
            "area": area,
            "bbox": (x_offset + x, y_offset + y, w, h),
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

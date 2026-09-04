"""detection.py has no ROS dependency, just opencv + numpy."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from mosquito_preference_assay.detection import find_candidates, parse_roi  # noqa: E402

_DEFAULTS = dict(diff_threshold=25, min_area=1, max_area=10000, morph_kernel=1)


def _fc(frame, bg, **overrides):
    kwargs = {**_DEFAULTS, **overrides}
    return find_candidates(frame, bg, **kwargs)


def _blank(h=100, w=120, value=50):
    return np.full((h, w), value, dtype=np.uint8)


def _with_blob(base, cx, cy, radius, value):
    img = base.copy()
    cv2.circle(img, (cx, cy), radius, int(value), -1)
    return img


def test_no_change_no_candidates():
    bg = _blank()
    assert _fc(bg, bg) == []


def test_detects_a_blob():
    bg = _blank()
    frame = _with_blob(bg, 60, 50, 6, 200)
    cands = _fc(frame, bg)
    assert len(cands) == 1
    c = cands[0]
    assert abs(c["cx"] - 60) < 2
    assert abs(c["cy"] - 50) < 2
    assert c["area"] > 0


def test_min_area_rejects_small_blob():
    bg = _blank()
    frame = _with_blob(bg, 60, 50, 2, 200)
    assert _fc(frame, bg, min_area=200) == []


def test_max_area_rejects_large_blob():
    bg = _blank()
    frame = _with_blob(bg, 60, 50, 30, 200)
    assert _fc(frame, bg, max_area=50) == []


def test_roi_excludes_blob_outside_it():
    bg = _blank()
    frame = _with_blob(bg, 10, 10, 5, 200)  # top-left corner
    assert _fc(frame, bg, roi=(50, 50, 120, 100)) == []
    # same blob, ROI now covers it
    assert len(_fc(frame, bg, roi=(0, 0, 30, 30))) == 1


def test_largest_first():
    bg = _blank()
    frame = _with_blob(bg, 30, 30, 3, 200)
    frame = _with_blob(frame, 90, 70, 8, 200)
    cands = _fc(frame, bg)
    assert len(cands) == 2
    assert cands[0]["area"] > cands[1]["area"]


def test_shape_mismatch_raises():
    with pytest.raises(ValueError):
        _fc(_blank(10, 10), _blank(20, 20))


def test_parse_roi():
    assert parse_roi("") is None
    assert parse_roi("  ") is None
    assert parse_roi("10,20,30,40") == (10, 20, 30, 40)
    with pytest.raises(ValueError):
        parse_roi("1,2,3")

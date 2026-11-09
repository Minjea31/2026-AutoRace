import numpy as np
import pytest

from autorace_crosswalk.white_roi import WhiteROI, validate_white


def test_white_ratio_uses_only_configured_roi():
    roi = WhiteROI()
    frame = np.full((200, 400, 3), 255, dtype=np.uint8)
    frame[172:184, 80:320] = 0
    ratio, bounds, mask = roi.measure(frame, 70, 120)
    assert bounds == (80, 172, 320, 184)
    assert ratio == 0 and mask.size == 240 * 12
    frame[172:184, 80:200] = 255
    ratio, _, _ = roi.measure(frame, 70, 120)
    assert ratio == pytest.approx(.5, abs=.02)


@pytest.mark.parametrize('colour, expected', [
    ((0, 255, 255), 0), ((119, 119, 119), 0), ((120, 120, 120), 1),
])
def test_lane_hsv_limits_reject_yellow_and_dark_floor(colour, expected):
    frame = np.empty((200, 400, 3), dtype=np.uint8)
    frame[:] = colour
    ratio, _, _ = WhiteROI().measure(frame, 70, 120)
    assert ratio == expected


def test_invalid_roi_and_thresholds_rejected():
    with pytest.raises(ValueError):
        WhiteROI(top=.95, bottom=.9)
    with pytest.raises(ValueError):
        validate_white(float('nan'), 120)

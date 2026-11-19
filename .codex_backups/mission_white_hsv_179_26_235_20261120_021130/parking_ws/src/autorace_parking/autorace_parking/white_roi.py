"""White pixel coverage in a shallow camera ROI, using lane HSV thresholds."""
# Same detector as autorace_slow.white_roi, kept local for standalone builds.
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_lane_white(path):
    config = yaml.safe_load(Path(path).read_text())['lane_lite']
    return validate_white(config['white_s_max'], config['white_v_min'])


def validate_white(saturation_max, value_min):
    values = (saturation_max, value_min)
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or not 0 <= v <= 255 for v in values):
        raise ValueError('Lane white HSV thresholds must be finite values from 0 to 255')
    return tuple(int(v) for v in values)


@dataclass(frozen=True)
class WhiteROI:
    left: float = .05
    right: float = .95
    top: float = .961
    bottom: float = 1.0
    minimum_ratio: float = .40

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.left, self.right, self.top, self.bottom, self.minimum_ratio))
                or not 0 <= self.left < self.right <= 1
                or not 0 <= self.top < self.bottom <= 1
                or not 0 < self.minimum_ratio <= 1):
            raise ValueError('Invalid normalized white ROI bounds or pixel ratio')

    def measure(self, image, saturation_max, value_min):
        height, width = image.shape[:2]
        x0, x1 = int(width * self.left), min(width, math.ceil(width * self.right))
        y0, y1 = int(height * self.top), min(height, math.ceil(height * self.bottom))
        crop = image[y0:y1, x0:x1]
        if crop.size == 0:
            raise ValueError('White ROI contains no pixels')
        # Match the lane colour threshold, before its lane-shape rejection filters.
        hsv = cv2.cvtColor(cv2.GaussianBlur(crop, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, value_min), (160, saturation_max, 255))
        ratio = float(np.count_nonzero(mask)) / mask.size
        return ratio, (x0, y0, x1, y1), mask

    def preview(self, image, bounds, ratio, mask, armed):
        output = image.copy()
        x0, y0, x1, y1 = bounds
        positive = ratio >= self.minimum_ratio
        colour = (90, 220, 110) if positive else (60, 190, 245)
        patch = output[y0:y1, x0:x1]
        tint = patch.copy()
        tint[mask > 0] = (90, 220, 110)
        output[y0:y1, x0:x1] = cv2.addWeighted(patch, .65, tint, .35, 0)
        cv2.rectangle(output, (x0, y0), (x1 - 1, y1 - 1), colour, 2)
        text = f'WHITE {ratio:.1%} / {self.minimum_ratio:.1%} | armed: {", ".join(armed) or "none"}'
        cv2.rectangle(output, (0, 0), (output.shape[1], 27), (20, 25, 25), -1)
        cv2.putText(output, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv2.LINE_AA)
        return output

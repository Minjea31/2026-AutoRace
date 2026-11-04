"""Small A/B-only OCR for white lettering on blue direction signs.

Uses reference glyph templates, not a general-purpose text OCR model.
Scores measure shape similarity and are not calibrated probabilities.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterReading:
    letter: str = ''
    score: float = 0.0
    reason: str = 'no readable A/B'


def normalize_glyph(mask):
    points = cv2.findNonZero(mask)
    canvas = np.zeros((64, 48), dtype=np.uint8)
    if points is None:
        return canvas
    x, y, w, h = cv2.boundingRect(points)
    scale = min(40 / w, 56 / h)
    glyph = cv2.resize(mask[y:y+h, x:x+w],
                       (max(1, round(w * scale)), max(1, round(h * scale))),
                       interpolation=cv2.INTER_NEAREST)
    gh, gw = glyph.shape
    canvas[(64-gh)//2:(64-gh)//2+gh, (48-gw)//2:(48-gw)//2+gw] = glyph
    return canvas


class DirectionOCR:
    def __init__(self, template_path, min_score=0.80, min_margin=0.08):
        with np.load(Path(template_path), allow_pickle=False) as templates:
            self.templates = {letter: templates[letter].astype(bool) for letter in ('A', 'B')}
        self.min_score = min_score
        self.min_margin = min_margin

    def read(self, sign):
        if sign.size == 0 or min(sign.shape[:2]) < 24:
            return LetterReading(reason='sign too small for OCR')
        h, w = sign.shape[:2]
        hsv = cv2.cvtColor(sign, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, (90, 70, 35), (140, 255, 255))
        if np.count_nonzero(blue) / (h * w) < 0.15:
            return LetterReading(reason='blue sign background not found')
        # Letters sit below the arrow. Exclude the white outer circle/background.
        crop = hsv[int(h*.38):int(h*.87), int(w*.12):int(w*.88)]
        mask = cv2.inRange(crop, (0, 0, 140), (180, 110, 255))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        readings = []
        best_score = 0.0
        for index in range(1, count):
            x, y, cw, ch, area = stats[index]
            if not (.12*w <= cw <= .48*w and .18*h <= ch <= .47*h):
                continue
            if area < .008*w*h or x == 0 or y == 0 or x+cw >= mask.shape[1] or y+ch >= mask.shape[0]:
                continue
            glyph = normalize_glyph((labels[y:y+ch, x:x+cw] == index).astype(np.uint8)*255).astype(bool)
            scores = {}
            for letter, templates in self.templates.items():
                intersection = np.count_nonzero(templates & glyph, axis=(1, 2))
                denominator = np.count_nonzero(templates, axis=(1, 2)) + np.count_nonzero(glyph)
                scores[letter] = float(np.max(2*intersection / np.maximum(denominator, 1)))
            ranking = sorted(scores, key=scores.get, reverse=True)
            best, second = ranking
            best_score = max(best_score, scores[best])
            if scores[best] >= self.min_score and scores[best]-scores[second] >= self.min_margin:
                readings.append(LetterReading(best, scores[best], 'matched'))
        if len({reading.letter for reading in readings}) > 1:
            return LetterReading(score=best_score, reason='ambiguous A/B')
        return max(readings, key=lambda reading: reading.score) if readings else LetterReading(
            score=best_score, reason='no confident A/B match')


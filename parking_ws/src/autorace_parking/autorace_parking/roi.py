"""Candidate-only ROI in the fixed parking-mission start frame."""
from copy import copy
from .tracking import compose


def contains(point,cfg):
    return bool(not cfg.candidate_roi_enabled or
            (cfg.candidate_roi_forward_min-1e-9<=point[0]<=cfg.candidate_roi_forward_max+1e-9 and
             cfg.candidate_roi_right_min-1e-9<=-point[1]<=cfg.candidate_roi_right_max+1e-9))


def candidate_view(scan,cfg,origin):
    if not cfg.candidate_roi_enabled:return scan
    points=[p for p in scan.points if contains(compose(origin,(*p,0.)),cfg)]
    if len(points)==len(scan.points):return scan
    result=copy(scan);result.points=points
    if hasattr(scan,'point_owners'):
        result.point_owners={tuple(p):scan.point_owners.get(tuple(p)) for p in points}
    return result

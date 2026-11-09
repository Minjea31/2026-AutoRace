"""Current yellow road edges in the deep mask; no accumulated vehicle pose."""
import math
import cv2
import numpy as np


def camera_road(mask, projection, c):
    from .node import MeasuredRoad
    if projection is None or min(mask.shape) < 2:
        return None
    h,w = mask.shape
    p = projection.c
    # Extend the calibrated BEV to the 3m ROI at half resolution so the outer
    # yellow edges remain visible beyond the narrow original lane crop.
    step = (p['bev_far_m']-p['bev_near_m'])/(projection.h-1)
    shift = (c.far_distance-p['bev_far_m'])/step/2
    extend = np.array([[.5,0,projection.w/4],[0,.5,shift],[0,0,1.]])
    transform = extend @ np.linalg.inv(projection.matrix) @ np.diag([1/(w-1),1/(h-1),1.])
    height = math.ceil((c.far_distance-p['bev_near_m'])/step/2)+1
    if height < 2:
        return None
    bev = cv2.warpPerspective(mask,transform,(projection.w,height),flags=cv2.INTER_NEAREST)
    # Class 1 includes yellow cone faces. Reject wide filled components in
    # metric BEV before fitting each separate thin paint component.
    count,labels=cv2.connectedComponents((bev == 1).astype(np.uint8),8)
    candidates={-1:[],1:[]}
    metres_per_col=2*p['lane_width_m']/p['lane_width_px']
    for label in range(1,count):
        rows,cols=np.nonzero(labels == label)
        if len(rows)<12:
            continue
        widths=[];samples=[]
        for row in np.unique(rows):
            cc=cols[rows==row]
            widths.append((cc.max()-cc.min()+1)*metres_per_col)
            samples.append((c.far_distance-row*step*2,
                (p['vehicle_center_u']*(projection.w-1)-(np.median(cc)-projection.w/4)*2)*p['lane_width_m']/p['lane_width_px']))
        if np.median(widths)>.20 or np.mean(np.asarray(widths)>.25)>.2:
            continue
        xx,yy=np.asarray(samples).T
        if len(xx)<8 or np.ptp(xx)<.25:
            continue
        a,b=np.polyfit(xx,yy,1)
        residual=np.abs(yy-(a*xx+b))
        if abs(a)>=1. or np.quantile(residual,.9)>.035 or abs(b)<.05:
            continue
        sign=1 if b>0 else -1
        candidates[sign].append((a,b))
    lines=[]
    for sign in (-1,1):
        if candidates[sign]:
            # The closest independently validated paint boundary on each side.
            lines.append(min(candidates[sign],key=lambda ab:abs(ab[1])/math.hypot(1.,ab[0])))
    if not lines:
        return None
    edges=[]
    for a,b in lines:
        scale=math.hypot(1.,a)
        inward=-1 if b>0 else 1
        edges.append((-inward*a/scale,inward/scale,-inward*b/scale))
    # Summary geometry is only for guidance/debug compatibility. Collision
    # checks use the independent measured half-planes above.
    a=float(np.mean([v[0] for v in lines]));scale=math.hypot(1.,a)
    b=float(np.mean([v[1] for v in lines]))
    half_width=abs(lines[1][1]-lines[0][1])/scale/2 if len(lines)==2 else 0.
    return MeasuredRoad(-a/scale,1/scale,-b/scale,half_width,2,tuple(edges))


def camera_guidance(road, c):
    if hasattr(road,'edges') and len(road.edges)<2:
        return 0.  # One observed boundary does not define a road centre.
    angle = c.lane_gain*(math.atan2(-road.offset,c.preview_distance)+math.atan2(-road.nx,road.ny))
    return max(-c.max_lane_steering,min(c.max_lane_steering,angle))

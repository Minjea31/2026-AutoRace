"""Known parked-LIMO footprint fitted behind the observed LiDAR surface.

Only registration consumes these boxes. Raw collision points are never moved.
A complete observed extent stays centered; an incomplete extent is anchored
on the face toward the sensor and completed away from the sensor.
"""
import math
import numpy as np


def infer_object_box(cluster,yaw,sensor,width,length,tolerance=.04):
    c,s=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[c,-s],[s,c]])
    points=np.asarray(cluster.points,dtype=float)@rotation
    lo,hi=points.min(axis=0),points.max(axis=0)
    observer=np.asarray(sensor)@rotation
    dimensions=(width,length)  # grid columns / grid depth; parked car faces lane
    center=(lo+hi)/2
    for axis,size in enumerate(dimensions):
        if hi[axis]-lo[axis] >= size-tolerance:
            continue
        if observer[axis]<lo[axis]-tolerance:
            center[axis]=lo[axis]+size/2
        elif observer[axis]>hi[axis]+tolerance:
            center[axis]=hi[axis]-size/2
    world=center@rotation.T
    corners=[(center+np.array((dx,dy)))@rotation.T
             for dx,dy in ((-width/2,-length/2),(width/2,-length/2),
                           (width/2,length/2),(-width/2,length/2),(-width/2,-length/2))]
    return dict(center=world.tolist(),corners=[p.tolist() for p in corners],
                track_id=cluster.track_id,width=width,length=length)

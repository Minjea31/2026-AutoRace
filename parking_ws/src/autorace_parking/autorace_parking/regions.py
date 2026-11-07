"""Solid occupied parking cells and rectangular vehicle footprint geometry."""
from copy import copy
import math


def rectangles_intersect(a,half_a,b,half_b):
    ax,ay,angle_a=a;bx,by,angle_b=b
    ca,sa=math.cos(angle_a),math.sin(angle_a);cb,sb=math.cos(angle_b),math.sin(angle_b)
    axes_a=((ca,sa),(-sa,ca));axes_b=((cb,sb),(-sb,cb));dx,dy=bx-ax,by-ay
    for x,y in axes_a+axes_b:
        ra=sum(h*abs(x*u+y*v) for h,(u,v) in zip(half_a,axes_a))
        rb=sum(h*abs(x*u+y*v) for h,(u,v) in zip(half_b,axes_b))
        if abs(x*dx+y*dy)>ra+rb+1e-9:return False
    return True


def with_occupied_slots(scan,layout,pose,cfg):
    """Attach filled cell rectangles, expressed in this scan's body frame."""
    if (not cfg.block_occupied_slots or not layout) and not getattr(scan,'occupied_slots',()):return scan
    result=copy(scan);result.occupied_slots=[]
    if not cfg.block_occupied_slots or not layout:return result
    c,s=math.cos(pose[2]),math.sin(pose[2]);angle=layout['yaw']-pose[2]
    for cell in layout['cells']:
        if not cell['occupied']:continue
        dx,dy=cell['center'][0]-pose[0],cell['center'][1]-pose[1]
        result.occupied_slots.append({'id':cell['id'],'pose':(c*dx+s*dy,-s*dx+c*dy,angle),
            'half_size':((layout['cell_width']+layout['border'])/2,(layout['cell_depth']+layout['border'])/2)})
    return result


def outside_grid(pose,grid,cfg):
    return not rectangles_intersect(pose,(cfg.vehicle_length/2+cfg.clearance,cfg.vehicle_width/2+cfg.clearance),
        (*grid['origin'],grid['yaw']),(grid['width']/2,grid['depth']/2))


def first_occupied_path_collision(checker,path,step):
    if not path:return None
    previous=path[0]
    if not checker.occupied_free(previous):return checker.last_block_detail
    for point in path[1:]:
        distance=math.hypot(point[0]-previous[0],point[1]-previous[1])
        angle=math.atan2(math.sin(point[2]-previous[2]),math.cos(point[2]-previous[2]))
        count=max(1,math.ceil(distance/step),math.ceil(abs(angle)/.05))
        for i in range(1,count+1):
            f=i/count;pose=(previous[0]+f*(point[0]-previous[0]),previous[1]+f*(point[1]-previous[1]),previous[2]+f*angle)
            if not checker.occupied_free(pose):return checker.last_block_detail
        previous=point
    return None

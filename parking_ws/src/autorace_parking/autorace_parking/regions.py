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


def selected_entry_slot(layout,slot_id=None,from_left_only=False):
    if not layout:return None
    candidates=[cell for cell in layout['cells'] if cell.get('eligible_empty') and not cell['occupied']]
    if slot_id is not None and slot_id>=0:
        candidates=[cell for cell in candidates if cell['id']==slot_id]
    if len(candidates)!=1:return None
    cell=candidates[0]
    slot={'slot_id':cell['id'],'pose':(*cell['center'],layout['yaw']),
          'half_size':(layout['cell_width']/2,layout['cell_depth']/2)}
    if from_left_only:
        cx,cy,yaw=slot['pose'];c,s=math.cos(yaw),math.sin(yaw)
        width,depth=layout['cell_width'],layout['cell_depth']
        normals=[('x_min',-c,-s,depth),('x_max',c,s,depth),
                 ('y_min',s,-c,width),('y_max',-s,c,width)]
        short=[edge for edge in normals if edge[3]<=min(width,depth)+1e-8]
        # +Y is the road/left side in the fixed parking-start frame. The
        # current vehicle pose must never change which side is the entrance.
        best=max(edge[2] for edge in short)
        facing=[edge for edge in short if edge[2]>=best-1e-8]
        chosen=max(facing,key=lambda edge:-edge[1]*cx-edge[2]*cy)
        slot['allowed_entry_edges']=(chosen[0],)
    return slot


def slot_entry_edges(layout,cfg,slot_id=None):
    """The selected empty bay's edges, without requiring body containment."""
    slot=selected_entry_slot(layout,slot_id,cfg.entry_from_left_only)
    if slot is None:return []
    cx,cy,yaw=slot['pose'];hw,hd=slot['half_size'];c,s=math.cos(yaw),math.sin(yaw)
    def world(x,y):return (cx+c*x-s*y,cy+s*x+c*y)
    edges=[('x_min',(-hw,-hd),(-hw,hd),2*hd),('x_max',(hw,-hd),(hw,hd),2*hd),
           ('y_min',(-hw,-hd),(hw,-hd),2*hw),('y_max',(-hw,hd),(hw,hd),2*hw)]
    return [{'edge':name,'a':world(*a),'b':world(*b),
             'open':name in slot['allowed_entry_edges'] if 'allowed_entry_edges' in slot else length<=2*min(hw,hd)+1e-8}
            for name,a,b,length in edges]


def invalid_slot_entry(slot,start,end):
    """Reject center-path entry via a bay's long edge; overhang is allowed."""
    if not slot:return None
    cx,cy,yaw=slot['pose'];c,s=math.cos(yaw),math.sin(yaw)
    def local(p):
        dx,dy=p[0]-cx,p[1]-cy
        return (c*dx+s*dy,-s*dx+c*dy)
    a,b=local(start),local(end);half=slot['half_size'];eps=1e-9
    if all(abs(a[i])<half[i]-eps for i in (0,1)):return None
    enter,leave=-math.inf,math.inf;faces=[]
    for axis in (0,1):
        delta=b[axis]-a[axis]
        if abs(delta)<eps:
            if abs(a[axis])>=half[axis]-eps:return None
            continue
        near=(-half[axis]-a[axis])/delta;far=(half[axis]-a[axis])/delta
        name=('x' if axis==0 else 'y')+('_min' if delta>0 else '_max')
        if near>far:near,far=far,near
        if near>enter+eps:enter,faces=near,[(axis,name)]
        elif abs(near-enter)<=eps:faces.append((axis,name))
        leave=min(leave,far)
    # A tangent or an outward crossing is not a new entry.
    if enter>leave+eps or min(leave,1.)<=max(enter,0.)+eps:return None
    allowed_axes={axis for axis in (0,1) if half[1-axis]<=min(half)+eps}
    permitted=slot.get('allowed_entry_edges')
    if any(axis in allowed_axes and (permitted is None or name in permitted) for axis,name in faces):return None
    fraction=max(0.,enter)
    angle=math.atan2(math.sin(end[2]-start[2]),math.cos(end[2]-start[2]))
    checked=[start[0]+fraction*(end[0]-start[0]),start[1]+fraction*(end[1]-start[1]),start[2]+fraction*angle]
    kind='slot_wrong_side_entry' if any(axis in allowed_axes for axis,_ in faces) else 'slot_long_side_entry'
    return {'kind':kind,'slot_id':slot['slot_id'],
            'edge':faces[0][1] if faces else 'long_side','checked_pose':checked}


def with_occupied_slots(scan,layout,pose,cfg,target_slot_id=None):
    """Attach filled cell rectangles, expressed in this scan's body frame."""
    if ((not cfg.block_occupied_slots or not layout) and not getattr(scan,'occupied_slots',())
            and not (cfg.entry_via_short_edge and layout) and not getattr(scan,'entry_slot',None)):return scan
    result=copy(scan);result.occupied_slots=[]
    result.entry_slot=None
    if cfg.entry_via_short_edge and layout:
        c,s=math.cos(pose[2]),math.sin(pose[2])
        slot=selected_entry_slot(layout,target_slot_id,cfg.entry_from_left_only)
        if slot is not None:
            dx,dy=slot['pose'][0]-pose[0],slot['pose'][1]-pose[1]
            result.entry_slot=dict(slot,pose=(c*dx+s*dy,-s*dx+c*dy,slot['pose'][2]-pose[2]))
    if not cfg.block_occupied_slots or not layout:return result
    c,s=math.cos(pose[2]),math.sin(pose[2]);angle=layout['yaw']-pose[2]
    for cell in layout['cells']:
        if not cell['occupied']:continue
        dx,dy=cell['center'][0]-pose[0],cell['center'][1]-pose[1]
        result.occupied_slots.append({'id':cell['id'],'pose':(c*dx+s*dy,-s*dx+c*dy,angle),
            'half_size':((cfg.slot_object_width/2,cfg.slot_object_length/2) if cfg.single_empty_slot else
                         ((layout['cell_width']+layout['border'])/2,(layout['cell_depth']+layout['border'])/2))})
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


def first_path_collision(checker,path,step):
    """Sweep a complete route against current points and configured regions."""
    if not path:return None
    previous=path[0]
    def blocked(pose,prior=None):
        if checker.free(pose) and (prior is None or checker.transition_free(prior,pose)):return None
        return checker.last_block_detail or {'kind':checker.last_block,'checked_pose':list(pose)}
    collision=blocked(previous)
    if collision:return collision
    for point in path[1:]:
        distance=math.hypot(point[0]-previous[0],point[1]-previous[1])
        angle=math.atan2(math.sin(point[2]-previous[2]),math.cos(point[2]-previous[2]))
        count=max(1,math.ceil(distance/step),math.ceil(abs(angle)/.05))
        prior=previous
        for i in range(1,count+1):
            f=i/count
            sample=(previous[0]+f*(point[0]-previous[0]),
                    previous[1]+f*(point[1]-previous[1]),previous[2]+f*angle)
            collision=blocked(sample,prior)
            if collision:return collision
            prior=sample
        previous=point
    return None

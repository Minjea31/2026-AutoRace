"""Parking pockets from left/right inner faces, an entrance, and a front face.

Geometry is expressed in the fixed mission frame, not the turning body frame.
Each boundary is an envelope of objects: duplicating/splitting observations does
not weight the target toward a more densely observed side.
"""
import math
from .core import wrap


def boundary_rectangles(objects,cfg,origin):
    result=[];seen=set();mission_row=cfg.row_yaw+origin[2]
    minimum=max(cfg.min_gap,cfg.vehicle_width+2*cfg.clearance)
    tolerance=cfg.boundary_tolerance
    for yaw in (mission_row,mission_row-math.pi/2,mission_row+math.pi/2,mission_row+math.pi):
        yaw=wrap(yaw);c,s=math.cos(yaw),math.sin(yaw)
        projected=[]
        for indices,cluster in objects:
            points=[(c*x+s*y,-s*x+c*y) for x,y in cluster.points]
            us,vs=zip(*points)
            projected.append({'indices':indices,'id':cluster.track_id if cluster.track_id is not None else -indices[0]-1,
                              'u0':min(us),'u1':max(us),'v0':min(vs),'v1':max(vs)})
        def overlaps(a,b,margin=0.):return a[1]>=b[0]-margin and b[1]>=a[0]-margin
        for left in projected:
            for right in projected:
                if left is right:continue
                width=left['v0']-right['v1']
                if not minimum-1e-9<=width<=cfg.max_gap+1e-9:continue
                if not overlaps((left['u0'],left['u1']),(right['u0'],right['u1']),cfg.row_tolerance):continue
                middle=(left['v0']+right['v1'])/2
                entry=max(left['u0'],right['u0'])
                for front in projected:
                    if front is left or front is right:continue
                    # A far corner alone is not a front obstacle. Its surface
                    # must reach the central vehicle corridor between the sides.
                    if not overlaps((front['v0'],front['v1']),
                                    (middle-cfg.vehicle_width/2,middle+cfg.vehicle_width/2)):continue
                    end=front['u0']
                    if end<=0. or not minimum<=end-entry<=2*cfg.enclosure_support_radius:continue
                    left_group=[p for p in projected if p is not front and abs(p['v0']-left['v0'])<=tolerance
                                and p['v0']>middle and overlaps((p['u0'],p['u1']),(entry,end))]
                    right_group=[p for p in projected if p is not front and abs(p['v1']-right['v1'])<=tolerance
                                 and p['v1']<middle and overlaps((p['u0'],p['u1']),(entry,end))]
                    if not left_group or not right_group:continue
                    vleft=min(p['v0'] for p in left_group);vright=max(p['v1'] for p in right_group)
                    middle=(vleft+vright)/2
                    front_group=[p for p in projected if p not in left_group and p not in right_group
                                 and abs(p['u0']-end)<=tolerance
                                 and overlaps((p['v0'],p['v1']),(middle-cfg.vehicle_width/2,middle+cfg.vehicle_width/2))]
                    if not front_group:continue
                    # The mouth starts where both side boundaries have begun.
                    entry=max(min(p['u0'] for p in left_group),min(p['u0'] for p in right_group))
                    end=min(p['u0'] for p in front_group)
                    width=vleft-vright;depth=end-entry
                    if not minimum-1e-9<=width<=cfg.max_gap+1e-9 or not minimum<=depth<=2*cfg.enclosure_support_radius:continue
                    u=(entry+end)/2;v=(vright+vleft)/2
                    center=(c*u-s*v,s*u+c*v)
                    if center[1]>=0:continue
                    support=left_group+right_group+front_group
                    if any(math.dist(((p['u0']+p['u1'])/2,(p['v0']+p['v1'])/2),(u,v))>cfg.enclosure_support_radius for p in support):continue
                    key=(round(yaw,6),round(entry,4),round(end,4),round(vright,4),round(vleft,4))
                    if key in seen:continue
                    seen.add(key)
                    def world(a,b):return (c*a-s*b,s*a+c*b)
                    roles={name:tuple(sorted(p['id'] for p in group)) for name,group in
                           [('left',left_group),('right',right_group),('front',front_group)]}
                    result.append({'center':center,'bay_yaw':yaw,'bounds':(entry,end,vright,vleft),
                        'width':width,'depth':depth,'support_ids':tuple(sorted({p['id'] for p in support})),
                        'indices':tuple(sorted({i for p in support for i in p['indices']})),
                        'roles':roles,'outline':[world(entry,vright),world(end,vright),world(end,vleft),world(entry,vleft)],
                        'entrance':[world(entry,vright),world(entry,vleft)],
                        'front':[world(end,vright),world(end,vleft)],
                        # A mouth facing the mission origin is preferred to a
                        # crosswise combination in a cluttered parking row.
                        'entry_distance':math.hypot(*world(entry,v))})
    # Rank geometry, never raw object counts or number of LiDAR returns.
    result.sort(key=lambda r:(r['entry_distance'],abs(r['width']-r['depth']),r['width']*r['depth']))
    return result

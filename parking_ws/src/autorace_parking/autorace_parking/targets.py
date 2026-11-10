"""Fixed geometric parking targets: enclosed empty-area center, then pair gap."""
from dataclasses import replace
import math

from .core import Cluster,Gap,CollisionChecker,find_gaps,wrap
from .enclosed import hull,physical_objects,inside
from .tracking import compose,inverse
from .roi import contains as roi_contains


SEMANTIC_GOALS=('enclosed_center','pair_midpoint')


def polygon_moments(polygon):
    if len(polygon)<3:return 0.,(0.,0.)
    cross=[a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1])]
    twice=sum(cross)
    if abs(twice)<1e-10:return 0.,(0.,0.)
    cx=sum((a[0]+b[0])*c for a,b,c in zip(polygon,polygon[1:]+polygon[:1],cross))/(3*twice)
    cy=sum((a[1]+b[1])*c for a,b,c in zip(polygon,polygon[1:]+polygon[:1],cross))/(3*twice)
    return abs(twice)/2,(cx,cy)


def clip_rectangle(polygon,bounds):
    """Intersection of a convex polygon with an axis-aligned rectangle."""
    xmin,xmax,ymin,ymax=bounds
    result=list(polygon)
    for axis,limit,keep_less in ((0,xmin,False),(0,xmax,True),(1,ymin,False),(1,ymax,True)):
        clipped=[]
        if not result:break
        def keep(p):return p[axis]<=limit+1e-10 if keep_less else p[axis]>=limit-1e-10
        previous=result[-1]
        for point in result:
            before,after=keep(previous),keep(point)
            if before!=after:
                fraction=(limit-previous[axis])/(point[axis]-previous[axis])
                clipped.append(tuple(previous[k]+fraction*(point[k]-previous[k]) for k in (0,1)))
            if after:clipped.append(point)
            previous=point
        result=clipped
    return result


def empty_area_center(polygon,objects):
    """Area centroid of the enclosing outline after removing known object boxes.

    Touching/overlapping tracked fragments have already been merged. A point
    chosen for easy entry never replaces this geometric final destination.
    """
    area,center=polygon_moments(polygon)
    sx,sy=area*center[0],area*center[1]
    # Integrate the union once, including overlapping remembered boxes. Use
    # disjoint vertical slabs so overlap cannot bias the empty-area centroid.
    boxes=[cluster.bounds for _,cluster in objects]
    xs=sorted({x for a,b,_,_ in boxes for x in (a,b)})
    for left,right in zip(xs,xs[1:]):
        intervals=sorted((a,b) for x1,x2,a,b in boxes if x1<(left+right)/2<x2)
        merged=[]
        for bottom,top in intervals:
            if merged and bottom<=merged[-1][1]:merged[-1]=(merged[-1][0],max(top,merged[-1][1]))
            else:merged.append((bottom,top))
        for bottom,top in merged:
            occupied,centroid=polygon_moments(clip_rectangle(polygon,(left,right,bottom,top)))
            area-=occupied;sx-=occupied*centroid[0];sy-=occupied*centroid[1]
    return (area,(sx/area,sy/area)) if area>1e-8 else (0.,center)


def box_pose_free(x,y,yaw,objects,cfg):
    """Separating-axis test against every known candidate object's full box."""
    c,s=math.cos(yaw),math.sin(yaw)
    length=cfg.vehicle_length/2+cfg.clearance
    width=cfg.vehicle_width/2+cfg.clearance
    body=[(x+c*dx-s*dy,y+s*dx+c*dy) for dx in (-length,length) for dy in (-width,width)]
    for _,cluster in objects:
        a,b,d,e=cluster.bounds;box=((a,d),(a,e),(b,d),(b,e))
        separate=False
        for ax,ay in ((1.,0.),(0.,1.),(c,s),(-s,c)):
            bp=[ax*px+ay*py for px,py in body];op=[ax*px+ay*py for px,py in box]
            if max(bp)<min(op)-1e-9 or max(op)<min(bp)-1e-9:
                separate=True;break
        if not separate:return False
    return True


def available_headings(center,objects,cfg,origin,checker):
    count=max(8,math.ceil(2*math.pi/cfg.target_heading_step))
    headings=[wrap(cfg.row_yaw+origin[2]+2*math.pi*k/count) for k in range(count)]
    headings.sort(key=lambda a:(abs(wrap(a-origin[2])),a))
    transform=inverse(origin);result=[]
    for angle in headings:
        if not box_pose_free(*center,angle,objects,cfg):continue
        local=compose(transform,(*center,angle))
        if checker.free(local,allow_unknown=True):result.append(local[2])
    return tuple(result)


def automatic_goals(scan,clusters,cfg,origin,details,map_hint=None):
    """Enclosure existence takes precedence, even when its center is blocked."""
    world=[Cluster([compose(origin,(*p,0.))[:2] for p in cluster.points],cluster.track_id) for cluster in clusters]
    if cfg.slot_map_enabled:
        from .slot_map import fit_slot_map,rotation
        merged=physical_objects(world,cfg,right_only=False)
        objects=[(indices,c) for indices,c in merged if cfg.min_cluster_size<=max(c.bounds[1]-c.bounds[0],c.bounds[3]-c.bounds[2])<=cfg.max_cluster_size]
        layout=fit_slot_map(objects,cfg,map_hint)
        details['slot_map']=layout;details['target_center_method']='six_slot_map'
        details['enclosure_objects']=len(objects);details['enclosure_detected']=layout is not None
        details['full_slot_map_matched']=layout is not None
        details['pair_map_hypotheses']=0
        goals=[]
        if layout is not None:
            from .regions import with_occupied_slots
            transform=inverse(origin)
            source=with_occupied_slots(scan,layout,origin,cfg)
            checker=CollisionChecker(source,cfg)
            for cell in layout['cells']:
                if not cell['eligible_empty'] or not roi_contains(cell['center'],cfg):continue
                if cfg.fixed_parking_slot_id>=0 and (cell['id']!=cfg.fixed_parking_slot_id or not layout['locked']):continue
                local=compose(transform,(*cell['center'],0.))
                options=available_headings(cell['center'],objects,cfg,origin,checker)
                goals.append(Gap(local[0],local[1],options[0] if options else 0.,cfg.slot_width,-1,-1,
                    support_ids=tuple(layout['support_ids']),confirmed=layout['locked'],kind='enclosed_center',
                    heading_options=options,stop_tolerance=cfg.center_stop_tolerance,
                    slot_id=cell['id'],bay_yaw=layout['yaw'],
                    parking_grid={'origin':layout['origin'],'yaw':layout['yaw'],
                        'width':cfg.slot_columns*cfg.slot_width+(cfg.slot_columns+1)*cfg.slot_border,
                        'depth':cfg.slot_rows*cfg.slot_depth+(cfg.slot_rows+1)*cfg.slot_border}))
        goals.sort(key=lambda g:(not bool(g.heading_options),g.x<0,math.hypot(g.x,g.y)))
        details['target_mode']='enclosed_center' if goals else 'none'
        details['enclosure_candidates']=len(goals)
        details['slot_map_reason']='ready' if goals else 'layout_not_matched'
        # Grid mode never starts parking from a pair or an unrelated enclosure.
        return goals
    merged=physical_objects(world,cfg)
    objects=[(indices,cluster) for indices,cluster in merged if cfg.min_cluster_size<=max(cluster.bounds[1]-cluster.bounds[0],cluster.bounds[3]-cluster.bounds[2])<=cfg.max_cluster_size]
    details['enclosure_objects']=len(objects);details['enclosure_detected']=False
    checker=CollisionChecker(scan,cfg);transform=inverse(origin)
    from .boundaries import boundary_rectangles
    rectangles=boundary_rectangles(objects,cfg,origin) if cfg.enclosed_space_detection else []
    if rectangles:
        goals=[];regions=[]
        for rect in rectangles[:6]:
            center=rect['center'];local=compose(transform,(*center,0.))
            options=available_headings(center,objects,cfg,origin,checker)
            indices=rect['indices'];ids=rect['support_ids']
            goals.append(Gap(local[0],local[1],options[0] if options else 0.,rect['width'],indices[0],indices[1],
                indices,ids,True,kind='enclosed_center',heading_options=options,stop_tolerance=cfg.center_stop_tolerance,
                bay_yaw=rect['bay_yaw'],boundary_bounds=rect['bounds']))
            regions.append({'center':local[:2],'support_ids':ids,'area_m2':rect['width']*rect['depth'],
                'outline':[compose(transform,(*p,0.))[:2] for p in rect['outline']],
                'entrance':[compose(transform,(*p,0.))[:2] for p in rect['entrance']],
                'front':[compose(transform,(*p,0.))[:2] for p in rect['front']],
                'boundary_roles':rect['roles'],'boundary_bounds':rect['bounds'],'bay_yaw':rect['bay_yaw'],
                'width_m':rect['width'],'depth_m':rect['depth'],'heading_count':len(options)})
        details['enclosure_detected']=True;details['target_mode']='enclosed_center'
        details['target_center_method']='three_boundaries'
        details['enclosure_candidates']=len(goals);details['center_regions']=regions
        return goals
    details['enclosure_candidates']=0
    if not cfg.pair_fallback_enabled:
        details['target_mode']='none';return []
    # Pair geometry still uses the configured row alignment, size and gap bounds.
    axes=[cfg.row_yaw]
    if cfg.slot_map_enabled:
        for i,(_,a) in enumerate(objects):
            for _,b in objects[i+1:]:
                if cfg.slot_initial_right_only and (a.center[1]>=0 or b.center[1]>=0):continue
                axes.append(wrap(math.atan2(b.center[1]-a.center[1],b.center[0]-a.center[0])-origin[2]))
        details['target_center_method']='two_object_gap'
    pairs=[];seen=set()
    for angle in axes:
        for candidate in find_gaps(clusters,replace(cfg,goal_from_observed_front=False,row_yaw=angle),origin,diagnostics=details):
            key=tuple(sorted((candidate.left,candidate.right)))
            if key not in seen:pairs.append(candidate);seen.add(key)
    accepted=[];checks=[]
    for pair in pairs:
        a,b=clusters[pair.left],clusters[pair.right]
        if a.track_id is not None and b.track_id is not None and a.track_id==b.track_id:continue
        # Two fragments belonging to one merged physical object are not a pair.
        if any(pair.left in ids and pair.right in ids for ids,_ in merged):continue
        center=compose(origin,(pair.x,pair.y,0.))[:2]
        options=available_headings(center,objects,cfg,origin,checker)
        checks.append({'track_ids':(a.track_id,b.track_id),'width_m':pair.width,'accepted':bool(options),
                       'reason':'' if options else 'goal_footprint_obstacle'})
        if not options:continue
        ids=tuple(sorted({i for i in (a.track_id,b.track_id) if i is not None}))
        accepted.append(replace(pair,yaw=options[0],support_indices=(pair.left,pair.right),support_ids=ids,
                                kind='pair_midpoint',heading_options=options,stop_tolerance=cfg.pair_stop_tolerance))
    details['candidate_checks']=checks;details['target_mode']='pair_midpoint' if accepted else 'none'
    details['pair_candidates']=len(accepted)
    return accepted


def update_geometry_goal(previous,candidates):
    """Never downgrade an enclosed center to an edge pair after occlusion."""
    if not candidates:return previous,False
    enclosed=[g for g in candidates if g.kind=='enclosed_center']
    if previous is None:return (enclosed or candidates)[0],True
    if previous.slot_id>=0 and any(g.slot_id>=0 for g in enclosed):
        matches=[g for g in enclosed if g.slot_id>=0 and math.hypot(g.x-previous.x,g.y-previous.y)<=.20]
        candidate=min(matches,key=lambda g:math.hypot(g.x-previous.x,g.y-previous.y)) if matches else enclosed[0]
    elif previous.kind=='enclosed_center':
        matches=[g for g in enclosed if (len(set(g.support_ids)&set(previous.support_ids))>=2 or math.hypot(g.x-previous.x,g.y-previous.y)<=.15)
                 and math.hypot(g.x-previous.x,g.y-previous.y)<=.8
                 and (g.bay_yaw is None or previous.bay_yaw is None or abs(wrap(g.bay_yaw-previous.bay_yaw))<.35)]
        if not matches:return previous,False
        candidate=min(matches,key=lambda g:math.hypot(g.x-previous.x,g.y-previous.y))
        # Boundary geometry, rather than a minimum object count, identifies the bay.
    elif enclosed:
        matches=[g for g in enclosed if set(previous.support_ids)<=set(g.support_ids)]
        candidate=min(matches,key=lambda g:math.hypot(g.x-previous.x,g.y-previous.y)) if matches else enclosed[0]
    else:
        matches=[g for g in candidates if set(g.support_ids)==set(previous.support_ids)]
        candidate=matches[0] if matches else previous
    if candidate.heading_options:
        preferred=min(candidate.heading_options,key=lambda a:abs(wrap(a-previous.yaw)))
        candidate=replace(candidate,yaw=preferred,heading_options=(preferred,)+tuple(a for a in candidate.heading_options if a!=preferred))
    candidate=replace(candidate,confirmed=previous.confirmed or candidate.confirmed) if previous.kind==candidate.kind else candidate
    changed=(candidate.kind!=previous.kind or math.hypot(candidate.x-previous.x,candidate.y-previous.y)>1e-6
             or candidate.confirmed!=previous.confirmed)
    return candidate,changed

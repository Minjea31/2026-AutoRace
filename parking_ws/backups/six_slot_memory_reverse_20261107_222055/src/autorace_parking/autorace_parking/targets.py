"""Fixed geometric parking targets: enclosed empty-area center, then pair gap."""
from dataclasses import replace
import math

from .core import Cluster,Gap,CollisionChecker,find_gaps,wrap
from .enclosed import hull,physical_objects,inside
from .tracking import compose,inverse


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


def automatic_goals(scan,clusters,cfg,origin,details):
    """Enclosure existence takes precedence, even when its center is blocked."""
    world=[Cluster([compose(origin,(*p,0.))[:2] for p in cluster.points],cluster.track_id) for cluster in clusters]
    merged=physical_objects(world,cfg)
    objects=[(indices,cluster) for indices,cluster in merged if cluster.track_id is not None
             and cfg.min_cluster_size<=max(cluster.bounds[1]-cluster.bounds[0],cluster.bounds[3]-cluster.bounds[2])<=cfg.max_cluster_size]
    details['enclosure_objects']=len(objects);details['enclosure_detected']=False
    checker=CollisionChecker(scan,cfg);transform=inverse(origin)
    region_goals=[];seen=set();regions=[]
    if cfg.enclosed_space_detection and len(objects)>=cfg.enclosure_min_objects:
        centers=[c.center for _,c in objects]
        seeds=centers+[((a[0]+b[0])/2,(a[1]+b[1])/2) for i,a in enumerate(centers) for b in centers[i+1:]]
        for seed in seeds:
            support=[o for o in objects if math.dist(seed,o[1].center)<=cfg.enclosure_support_radius]
            if len(support)<cfg.enclosure_min_objects:continue
            ids=tuple(sorted(o[1].track_id for o in support))
            if ids in seen:continue
            seen.add(ids);outline=hull([c.center for _,c in support]);area,_=polygon_moments(outline)
            if area<cfg.vehicle_length*cfg.vehicle_width:continue
            empty_area,center=empty_area_center(outline,support)
            if empty_area<cfg.vehicle_length*cfg.vehicle_width or not inside(center,outline):continue
            # Reject thin, nearly collinear rows that do not surround a bay.
            angles=sorted(math.atan2(c.center[1]-center[1],c.center[0]-center[0]) for _,c in support)
            gaps=[(angles[(i+1)%len(angles)]-a)%(2*math.pi) for i,a in enumerate(angles)]
            if max(gaps)>=math.pi-.02:continue
            local=compose(transform,(*center,origin[2]));options=available_headings(center,objects,cfg,origin,checker)
            indices=tuple(i for inds,_ in support for i in inds)
            distance=min(math.hypot(max(c.bounds[0]-center[0],0.,center[0]-c.bounds[1]),
                                    max(c.bounds[2]-center[1],0.,center[1]-c.bounds[3])) for _,c in objects)
            goal=Gap(local[0],local[1],options[0] if options else 0.,2*distance,indices[0],indices[1],indices,ids,
                     len(ids)>=cfg.enclosure_confirm_objects,kind='enclosed_center',heading_options=options,
                     stop_tolerance=cfg.center_stop_tolerance)
            region_goals.append((goal,empty_area,outline))
        # A complete enclosure supersedes triangular subregions of that bay.
        maximal=[r for r in region_goals if not any(set(r[0].support_ids)<set(other[0].support_ids) for other in region_goals)]
        maximal.sort(key=lambda r:(not r[0].confirmed,-len(r[0].support_ids),r[1],math.hypot(*compose(origin,(r[0].x,r[0].y,0.))[:2])))
        if maximal:
            details['enclosure_detected']=True;details['target_mode']='enclosed_center'
            details['enclosure_candidates']=len(maximal)
            details['center_regions']=[{'center':(g.x,g.y),'support_ids':g.support_ids,'area_m2':area,
                 'outline':[compose(transform,(*p,0.))[:2] for p in outline], 'heading_count':len(g.heading_options)} for g,area,outline in maximal]
            return [g for g,_,_ in maximal]
    details['enclosure_candidates']=0
    if not cfg.pair_fallback_enabled:
        details['target_mode']='none';return []
    # Pair geometry still uses the configured row alignment, size and gap bounds.
    pairs=find_gaps(clusters,replace(cfg,goal_from_observed_front=False),origin,diagnostics=details)
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
    if previous.kind=='enclosed_center':
        matches=[g for g in enclosed if len(set(g.support_ids)&set(previous.support_ids))>=min(3,len(previous.support_ids))
                 and math.hypot(g.x-previous.x,g.y-previous.y)<=.8]
        if not matches:return previous,False
        candidate=min(matches,key=lambda g:math.hypot(g.x-previous.x,g.y-previous.y))
        if previous.confirmed and len(candidate.support_ids)<len(previous.support_ids):return previous,False
    elif enclosed:
        candidate=next((g for g in enclosed if set(previous.support_ids)<=set(g.support_ids)),enclosed[0])
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

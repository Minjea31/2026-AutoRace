"""Find vehicle-sized empty centers supported by surrounding tracked objects."""
import math
from .core import Gap,Cluster,CollisionChecker,wrap
from .tracking import compose,inverse


def hull(points):
    points=sorted(set(points))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def inside(point,polygon):
    return len(polygon)>=3 and all((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])>=-1e-9
                                  for a,b in zip(polygon,polygon[1:]+polygon[:1]))


def physical_objects(clusters,cfg,right_only=True):
    """Combine touching tracked fragments before counting surrounding objects.

    A wall remembered under several IDs must not confirm a five-object bay.
    Keep original indices so the selected geometry can still be displayed.
    """
    groups=[]
    margin=min(.08,cfg.cluster_distance*2/3)
    def adjacent(a,b):
        ax,bx,ay,by=a.bounds;cx,dx,cy,dy=b.bounds
        return math.hypot(max(ax-dx,cx-bx,0.),max(ay-dy,cy-by,0.))<=margin
    for i,c in enumerate(clusters):
        if right_only and c.center[1]>=0:continue
        members=[(i,c)];remaining=[]
        for group in groups:
            if any(adjacent(c,other) for _,other in group):members.extend(group)
            else:remaining.append(group)
        groups=remaining+[members]
    return [(tuple(i for i,_ in group),Cluster([p for _,c in group for p in c.points],
             min((c.track_id for _,c in group if c.track_id is not None),default=None))) for group in groups]


def enclosed_goals(scan,clusters,cfg,origin,details):
    # Evaluate empty-space geometry in the fixed mission frame. Recomputing
    # axis-aligned object bounds in the turning vehicle frame shifts the goal.
    clusters=[Cluster([compose(origin,(*p,0.))[:2] for p in cluster.points],cluster.track_id)
              for cluster in clusters]
    transform=inverse(origin)
    usable=[]
    indices_by_object={}
    for indices,cluster in physical_objects(clusters,cfg):
        i=indices[0];indices_by_object[i]=indices
        x,y=cluster.center
        size=max(cluster.bounds[1]-cluster.bounds[0],cluster.bounds[3]-cluster.bounds[2])
        mission_y=y
        if mission_y<0 and cfg.min_cluster_size<=size<=cfg.max_cluster_size:
            usable.append((i,cluster))
    details['enclosure_objects']=len(usable)
    if len(usable)<cfg.enclosure_min_objects:return []
    xmin,xmax=min(c.center[0] for _,c in usable),max(c.center[0] for _,c in usable)
    ymin,ymax=min(c.center[1] for _,c in usable),max(c.center[1] for _,c in usable)
    step=max(cfg.enclosure_grid_resolution,math.sqrt(max(0.,(xmax-xmin)*(ymax-ymin))/2500.))
    ranked=[]
    hulls={}
    yaw=cfg.row_yaw+origin[2]-math.pi/2
    co,si=math.cos(yaw),math.sin(yaw)
    corners=[(co*x-si*y,si*x+co*y)
             for x in (-(cfg.vehicle_length/2+cfg.clearance),cfg.vehicle_length/2+cfg.clearance)
             for y in (-(cfg.vehicle_width/2+cfg.clearance),cfg.vehicle_width/2+cfg.clearance)]
    for ix in range(int((xmax-xmin)/step)+1):
        x=xmin+ix*step
        for iy in range(int((ymax-ymin)/step)+1):
            y=ymin+iy*step
            support=[(i,c) for i,c in usable if math.hypot(x-c.center[0],y-c.center[1])<=cfg.enclosure_support_radius]
            if len(support)<cfg.enclosure_min_objects:continue
            angles=sorted(math.atan2(c.center[1]-y,c.center[0]-x) for _,c in support)
            gaps=[(angles[(k+1)%len(angles)]-angles[k])%(2*math.pi) for k in range(len(angles))]
            if max(gaps)>math.pi-.02:continue  # Supports must surround, not all lie on one side.
            key=tuple(i for i,_ in support)
            if key not in hulls:hulls[key]=hull([c.center for _,c in support])
            if not all(inside((x+dx,y+dy),hulls[key]) for dx,dy in corners):continue
            distances=[]
            for _,c in usable:
                a,b,d,e=c.bounds
                distances.append(math.hypot(max(a-x,0.,x-b),max(d-y,0.,y-e)))
            clearance=min(distances)
            if not max(cfg.min_gap,cfg.vehicle_width+2*cfg.clearance)<=2*clearance<=cfg.max_gap:continue
            ids=tuple(sorted({c.track_id for _,c in support if c.track_id is not None}))
            if len(ids)<cfg.enclosure_min_objects:continue
            score=clearance-.015*math.hypot(x-origin[0],y-origin[1])
            ranked.append((score,x,y,clearance,tuple(j for i,_ in support for j in indices_by_object[i]),ids))
    checker=CollisionChecker(scan,cfg);result=[]
    for score,x,y,clearance,indices,ids in sorted(ranked,reverse=True):
        if any(math.hypot(x-compose(origin,(g.x,g.y,g.yaw))[0],y-compose(origin,(g.x,g.y,g.yaw))[1])<.15 for g in result):continue
        yaw=wrap(cfg.row_yaw+origin[2]-math.pi/2)
        local=compose(transform,(x,y,yaw))
        if not checker.free(local,allow_unknown=True):continue
        result.append(Gap(*local,2*clearance,indices[0],indices[1],indices,ids,len(ids)>=cfg.enclosure_confirm_objects))
        if len(result)>=6:break
    details['enclosure_candidates']=len(result)
    return result

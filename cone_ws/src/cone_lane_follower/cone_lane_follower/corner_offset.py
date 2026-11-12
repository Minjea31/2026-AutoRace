"""Ordered parallel-line offsets with loop trimming and bounded-curvature corners."""
import math
from cone_lane_follower.async_planner import check_budget


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def curvature(path):
    values = []
    for a, b, c in zip(path, path[1:], path[2:]):
        u = (b[0]-a[0], b[1]-a[1])
        v = (c[0]-b[0], c[1]-b[1])
        denominator = math.hypot(*u)*math.hypot(*v)*math.dist(a[:2], c[:2])
        values.append(2*abs(cross(u, v))/denominator if denominator > 1e-12 else math.inf)
    return values


def offset(boundary, distance, step, minimum_radius, api, parameters, diagnostics):
    """Keep the observed side; never extrapolate past offset boundary endpoints."""
    segments = []
    for a, b in zip(boundary, boundary[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        t = (dx/length, dy/length)
        q = (a[0]-distance*t[1], a[1]+distance*t[0], a[2])
        end = (b[0]-distance*t[1], b[1]+distance*t[0], b[2])
        segments.append((q, end, t))
    if not segments:
        diagnostics['corner_failure'] = 'no nonzero boundary segments'
        return []
    vertices = [segments[0][0]]
    corners = 0
    for previous, current in zip(segments, segments[1:]):
        a, end, u = previous
        b, _, v = current
        cosine = u[0]*v[0]+u[1]*v[1]
        if cosine <= -.95:
            diagnostics['corner_failure'] = 'boundary reversal; check cone association'
            return []
        denominator = cross(u, v)
        if abs(denominator) < 1e-8:
            vertices.append(b)
            continue
        corners += math.acos(max(-1., min(1., cosine))) > math.radians(15)
        t = cross((b[0]-a[0], b[1]-a[1]), v)/denominator
        point = (a[0]+t*u[0], a[1]+t*u[1], b[2])
        if math.dist(point[:2], end[:2]) > max(abs(distance)*4, step):
            diagnostics['corner_failure'] = 'unbounded miter; check cone association'
            return []
        vertices.append(point)
    vertices.append(segments[-1][1])
    # Remove loops created by inward offsets, maintaining traversal order.
    trimmed = 0
    while True:
        hit = None
        for i, (a,b) in enumerate(zip(vertices,vertices[1:])):
            for j in range(i+2,len(vertices)-1):
                c,d = vertices[j:j+2]
                u=(b[0]-a[0],b[1]-a[1]); v=(d[0]-c[0],d[1]-c[1])
                denom=cross(u,v)
                if abs(denom)<1e-10: continue
                delta=(c[0]-a[0],c[1]-a[1])
                t=cross(delta,v)/denom; z=cross(delta,u)/denom
                if 0<=t<=1 and 0<=z<=1:
                    hit=(i,j,(a[0]+t*u[0],a[1]+t*u[1],a[2]));break
            if hit:break
        if not hit:break
        i,j,q=hit
        vertices=vertices[:i+1]+[q]+vertices[j+1:];trimmed+=1
    clean=[]
    for q in vertices:
        if not clean or math.dist(q,clean[-1])>1e-8:clean.append(q)
    path=api.sample_boundary(clean,step)
    diagnostics['corners'] = int(corners)
    diagnostics['trimmed_loops'] = trimmed
    side=1. if distance>0 else -1.
    def acceptable(candidate):
        if not api.offset_safe(candidate,[(boundary,side)]) or not api.valid_center_geometry(candidate,parameters):
            return False
        # Progress is measured on the observed boundary, never by increasing X.
        progress = -math.inf
        for point in candidate:
            hits = []
            arc = 0.
            for a,b in zip(boundary,boundary[1:]):
                length = math.dist(a[:2],b[:2])
                if length < 1e-9:continue
                dist,t,q = api.segment_projection(point,a,b)
                hits.append((dist,arc+t*length))
                arc += length
            if not hits:return False
            current = min(hits)[1]
            if current < progress-step:return False
            progress = max(progress,current)
        return True
    limit=1/minimum_radius
    # Local quintic replacements use progressively larger observed windows.
    for iteration in range(12):
        check_budget(parameters)
        values=curvature(path)
        peak=max(values,default=0.)
        if peak <= limit and acceptable(path):
            diagnostics['max_curvature_per_m']=peak
            diagnostics['min_radius_m']=minimum_radius
            diagnostics['corner_smoothing']='success'
            return path
        if len(path)<4:break
        index=values.index(peak)+1 if values else len(path)//2
        best=None
        for span in range(3,len(path)):
            check_budget(parameters)
            lo=max(0,index-span);hi=min(len(path)-1,index+span)
            if hi-lo<3:continue
            bridge=api.quintic_bridge(path,lo,path,hi,step)
            candidate=path[:lo]+bridge+path[hi+1:]
            if not bridge or not acceptable(candidate):continue
            new_peak=max(curvature(candidate),default=0.)
            if new_peak<peak-1e-6 and (best is None or new_peak<best[0]):
                best=(new_peak,candidate)
            if new_peak<=limit:break
        if best is None:break
        path=best[1]
    diagnostics['corner_failure']=f'no safe corner within radius >= {minimum_radius:.3f}m'
    diagnostics['max_curvature_per_m']=max(curvature(path),default=0.)
    return []

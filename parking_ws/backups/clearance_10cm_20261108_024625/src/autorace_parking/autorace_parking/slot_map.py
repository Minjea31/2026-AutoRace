"""Rigid metric parking-grid registration in the odometry-anchored mission frame.

Objects provide occupied-cell evidence, not a vote on the center of empty space.
With partial observations, only empty cells shared by plausible grid placements
are eligible targets. A locked placement persists through LiDAR occlusion.
"""
import math
import numpy as np
from .core import wrap


def reference_centers(cfg):
    return np.array([((col-(cfg.slot_columns-1)/2)*(cfg.slot_width+cfg.slot_border),
                      (row-(cfg.slot_rows-1)/2)*(cfg.slot_depth+cfg.slot_border))
                     for row in range(cfg.slot_rows) for col in range(cfg.slot_columns)])


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw);return np.array([[c,-s],[s,c]])


def fit_slot_map(objects,cfg,hint=None):
    reference=reference_centers(cfg)
    points=np.array([cluster.center for _,cluster in objects],dtype=float).reshape(-1,2)
    owners=[cluster.track_id if cluster.track_id is not None else -indices[0]-1 for indices,cluster in objects]
    gate=cfg.slot_match_tolerance
    def evaluate(center,yaw):
        centers=reference@rotation(yaw).T+center
        distances=np.linalg.norm(centers[:,None,:]-points[None,:,:],axis=2) if len(points) else np.empty((len(reference),0))
        nearest=np.argmin(distances,axis=1) if len(points) else np.zeros(len(reference),dtype=int)
        errors=distances[np.arange(len(reference)),nearest] if len(points) else np.full(len(reference),math.inf)
        matches=errors<=gate
        return centers,nearest,matches,float(np.sqrt(np.mean(errors[matches]**2))) if np.any(matches) else math.inf
    locked=bool(hint and hint.get('locked'))
    if locked:
        center=np.array(hint['origin'],dtype=float);yaw=hint['yaw']
        centers,nearest,matches,error=evaluate(center,yaw)
        # Odometry anchors the placement. Missing returns never move the map
        # or turn a previously observed occupied cell into an empty one.
        alternatives=[{'origin':center,'yaw':yaw,'centers':centers}]
        count=int(matches.sum());extent_locked=True
    else:
        if len(points)<3:return None
        seeds={}
        for i in range(len(points)):
            for j in range(i+1,len(points)):
                dv=points[j]-points[i];length=np.linalg.norm(dv)
                for a in range(len(reference)):
                    for b in range(len(reference)):
                        if a==b:continue
                        delta=reference[b]-reference[a]
                        if abs(float(np.linalg.norm(delta))-length)>gate*1.5:continue
                        yaw=wrap(math.atan2(dv[1],dv[0])-math.atan2(delta[1],delta[0]))
                        # A 180-degree flip is the same physical grid.
                        if yaw>=math.pi/2:yaw-=math.pi
                        if yaw< -math.pi/2:yaw+=math.pi
                        center=points[i]-rotation(yaw)@reference[a]
                        key=(round(center[0]/.03),round(center[1]/.03),round(yaw/math.radians(2)))
                        seeds[key]=(center,yaw)
        if not seeds:return None
        values=list(seeds.values());origins=np.array([a for a,_ in values]);angles=np.array([b for _,b in values])
        co,si=np.cos(angles),np.sin(angles)
        predicted=np.stack((co[:,None]*reference[None,:,0]-si[:,None]*reference[None,:,1],
                            si[:,None]*reference[None,:,0]+co[:,None]*reference[None,:,1]),axis=2)+origins[:,None,:]
        distances=np.linalg.norm(predicted[:,:,None,:]-points[None,None,:,:],axis=3)
        nearest=np.argmin(distances,axis=2);errors=np.take_along_axis(distances,nearest[:,:,None],axis=2)[:,:,0]
        matched=errors<=gate;counts=matched.sum(axis=1)
        rms=np.sqrt(np.sum(np.where(matched,errors**2,0.),axis=1)/np.maximum(1,counts))
        order=np.lexsort((rms,-counts));candidates=[]
        diverse=[]
        for index in order:
            if len(diverse)>=64:break
            center,yaw=values[index]
            if any(np.linalg.norm(center-a)<.10 and abs(wrap(yaw-b))<math.radians(5) for a,b in diverse):continue
            diverse.append((center,yaw))
            if counts[index]<3:break
            center,yaw=values[index]
            for _ in range(3):
                centers,association,matches,error=evaluate(center,yaw)
                src=reference[matches];dst=points[association[matches]]
                if len(src)<3 or np.linalg.matrix_rank(src-src.mean(axis=0),tol=.01)<2:break
                # Gate < half the pitch guarantees distinct cell/point matches.
                u,_,vt=np.linalg.svd((src-src.mean(axis=0)).T@(dst-dst.mean(axis=0)))
                rot=vt.T@u.T
                if np.linalg.det(rot)<0:vt[-1,:]*=-1;rot=vt.T@u.T
                yaw=math.atan2(rot[1,0],rot[0,0]);center=dst.mean(axis=0)-rot@src.mean(axis=0)
            centers,association,matches,error=evaluate(center,yaw)
            if int(matches.sum())<3 or error>cfg.slot_fit_max_error:continue
            if cfg.slot_initial_right_only and center[1]>=0:continue
            if np.linalg.matrix_rank(reference[matches]-reference[matches].mean(axis=0),tol=.01)<2:continue
            if any(np.max(np.min(np.linalg.norm(centers[:,None,:]-v['centers'][None,:,:],axis=2),axis=1))<.06 for v in candidates):continue
            candidates.append({'origin':center,'yaw':yaw,'centers':centers,'matches':int(matches.sum()),'error':error})
        if not candidates:return None
        candidates.sort(key=lambda a:(-a['matches'],a['error']))
        best=candidates[0];count=best['matches'];error=best['error']
        alternatives=[v for v in candidates if v['matches']==count and v['error']<=error+cfg.slot_ambiguity_margin]
        center=best['origin'];yaw=best['yaw'];centers=best['centers'];extent_locked=len(alternatives)==1
    old_cells={c['id']:c for c in hint.get('cells',[])} if locked else {}
    cells=[];rot=rotation(yaw)
    for index,world in enumerate(centers):
        coordinates=(points-world)@rot if len(points) else np.empty((0,2))
        occupied=[owners[i] for i,p in enumerate(coordinates) if abs(p[0])<=cfg.slot_width/2 and abs(p[1])<=cfg.slot_depth/2]
        if old_cells.get(index,{}).get('occupied'):
            occupied=sorted(set(occupied)|set(old_cells[index].get('occupant_ids',[])))
        on_right=bool(not cfg.slot_initial_right_only or world[1]<0)
        inside_all=all(float(np.min(np.linalg.norm(world-v['centers'],axis=1)))<=gate for v in alternatives)
        cells.append({'id':index,'column':index%cfg.slot_columns,'row':index//cfg.slot_columns,
                      'center':world.tolist(),'occupied':bool(occupied),'occupant_ids':occupied,
                      'eligible_empty':not occupied and inside_all and on_right,
                      'state':'occupied' if occupied else ('outside_initial_right' if not on_right else ('empty_candidate' if inside_all else 'unconfirmed'))})
    return {'origin':center.tolist(),'yaw':float(yaw),'locked':bool(extent_locked),'matched_cells':count,
            'fit_rmse_m':float(error) if math.isfinite(error) else None,'hypotheses':len(alternatives),
            'columns':cfg.slot_columns,'rows':cfg.slot_rows,'cell_width':cfg.slot_width,'cell_depth':cfg.slot_depth,
            'border':cfg.slot_border,'cells':cells,'support_ids':sorted({i for cell in cells for i in cell['occupant_ids']}),
            'frame':'mission','method':'metric_six_slot_template'}

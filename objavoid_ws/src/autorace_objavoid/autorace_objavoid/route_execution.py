"""Execute the same two steering segments that the planner collision-checks."""
from dataclasses import dataclass, replace
import math


class MotionDistance:
    def __init__(self):
        self.last_pose = None
        self.sample = (0.,0,None)  # total forward travel, reset epoch, monotonic receive time

    def update(self,x,y,stamp,received):
        if not all(math.isfinite(v) for v in (x,y,stamp,received)):
            return
        distance,epoch,_ = self.sample
        if self.last_pose is not None:
            px,py,pt = self.last_pose
            dt=stamp-pt
            step=math.hypot(x-px,y-py)
            if dt==0:
                return  # Duplicate packet is not a fresh pose or a reset.
            if dt<0 or dt>.5 or step>.25:
                epoch+=1  # A clock/pose reset must not complete an old turn.
            else:
                distance+=step
        self.last_pose=(x,y,stamp)
        self.sample=(distance,epoch,received)

    def snapshot(self,now,timeout):
        distance,epoch,received=self.sample
        if received is None or max(0.,now-received)>=timeout:
            return None
        return distance,epoch


@dataclass
class RouteExecution:
    result: tuple | None = None
    origin: float = 0.
    epoch: int = 0
    primary: tuple | None = None
    preference: int = 0

    def reset(self):
        self.result=None
        self.primary=None

    def begin(self,result,motion,targets,preference):
        self.result=result
        self.origin,self.epoch=motion
        self.primary=targets[0] if targets else None
        self.preference=preference

    def matches(self,targets,preference):
        return (self.result is not None and bool(targets) and self.primary is not None
                and preference==self.preference
                and math.dist(targets[0][:2],self.primary[:2])<.30
                and abs(targets[0][1]-self.primary[1])<.15)

    def remaining(self,motion):
        if self.result is None or motion is None:
            return None
        distance,epoch=motion
        state,speed,angle,c=self.result
        progress=distance-self.origin
        if epoch!=self.epoch or progress<0 or progress>=c.preview_distance-.025:
            self.reset()
            return None
        split=c.preview_distance if c.turn_distance<0 else c.turn_distance
        # Switch in the last control interval before the junction, then validate
        # the new tail from the current pose. This avoids holding the large turn
        # for another complete timer period after crossing the junction.
        lead=speed/c.publish_rate if split<c.preview_distance else 0.
        if progress<split-lead:
            route=replace(c,preview_distance=c.preview_distance-progress,
                          turn_distance=split-progress)
            return state+'/TURN',speed,angle,route
        route=replace(c,preview_distance=c.preview_distance-progress,
                      turn_distance=-1.,follow_steering=0.)
        return state+'/FOLLOW',speed,c.follow_steering,route

    def checked_remaining(self,motion,is_clear):
        result=self.remaining(motion)
        if result is not None and not is_clear(result[2],result[3]):
            self.reset()
            return None
        return result

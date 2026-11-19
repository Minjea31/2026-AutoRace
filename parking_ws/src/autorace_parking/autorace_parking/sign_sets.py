"""Emit one confirmation per stream of parking true messages."""
import math
class ParkingSignSet:
    def __init__(self,gap_sec=1.0,required=2):
        if not math.isfinite(gap_sec) or gap_sec<=0:raise ValueError('parking_sign_set_gap_sec must be positive and finite')
        if isinstance(required,bool) or not isinstance(required,int) or required<1:raise ValueError('recognition_frames must be positive integer')
        self.gap_sec,self.required=gap_sec,required
        self.last_true=None
        self.count=0
        self.confirmed=False
    def receive(self,detected,wall,required=None):
        if not detected:return False
        if required is not None:self.required=required
        if self.last_true is None or wall<self.last_true or wall-self.last_true>self.gap_sec:
            self.count=0
            self.confirmed=False
        self.last_true=wall
        self.count=min(self.required,self.count+1)
        if not self.confirmed and self.count>=self.required:
            self.confirmed=True
            return True
        return False

"""Observer-only orbit/pan/zoom state; independent of vehicle sensor extrinsics."""
import math
import numpy as np


class OrbitView:
    def __init__(self,eye=(6.9,2.4,3.2),target=(6.65,4.45,0)):
        self.set_pose(eye,target)

    def set_pose(self,eye,target):
        self.target = np.asarray(target,dtype=float).copy()
        offset = np.asarray(eye,dtype=float)-self.target
        self.distance = max(.2,float(np.linalg.norm(offset)))
        self.azimuth = math.atan2(offset[1],offset[0])
        self.elevation = math.asin(float(np.clip(offset[2]/self.distance,-1,1)))

    def eye(self):
        c = math.cos(self.elevation)
        return self.target+self.distance*np.array([c*math.cos(self.azimuth),c*math.sin(self.azimuth),math.sin(self.elevation)])

    def rotate(self,dx,dy):
        self.azimuth -= dx*math.pi
        self.elevation = float(np.clip(self.elevation+dy*math.pi/2,math.radians(5),math.radians(89.5)))

    def pan(self,dx,dy,hfov,vfov):
        right = np.array([-math.sin(self.azimuth),math.cos(self.azimuth),0.])
        forward = np.array([-math.cos(self.azimuth),-math.sin(self.azimuth),0.])
        self.target -= self.distance*(right*dx*math.tan(math.radians(hfov)/2)
                                      +forward*dy*math.tan(math.radians(vfov)/2))

    def zoom(self,steps):
        self.distance = float(np.clip(self.distance*math.exp(-steps*.12),.2,30.))

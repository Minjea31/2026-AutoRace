"""Finite yellow segments; inspect only the overlapping part of the body."""
import math


class YellowBoundary:
    def __init__(self, segments):
        self.segments = []
        for a,b,side in segments:
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            if length>=.02:
                self.segments.append((a[0],a[1],dx/length,dy/length,length,side))

    @staticmethod
    def clipped_crosses(along,distance,ux,uy,vx,vy,length):
        # Rectangle in segment coordinates: u along the observed segment,
        # v positive on the permitted side. Clip to 0 <= u <= length before
        # testing v; a corner beyond either endpoint must not reject the path.
        corners=((along-ux-vx,distance-uy-vy),
                 (along+ux-vx,distance+uy-vy),
                 (along+ux+vx,distance+uy+vy),
                 (along-ux+vx,distance-uy+vy))
        previous=corners[-1]
        for current in corners:
            u,v=current
            if 0<=u<=length and v < -1e-6:
                return True
            pu,pv=previous
            if abs(u-pu)>1e-12:
                for end in (0.,length):
                    fraction=(end-pu)/(u-pu)
                    if 0<=fraction<=1 and pv+fraction*(v-pv) < -1e-6:
                        return True
            previous=current
        return False

    def crosses(self,x,y,c,s,half_length,half_width):
        for ax,ay,tx,ty,length,side in self.segments:
            dx,dy=x-ax,y-ay
            along=dx*tx+dy*ty
            ux=half_length*(c*tx+s*ty)
            vx=half_width*(-s*tx+c*ty)
            extent=abs(ux)+abs(vx)
            if along+extent<0 or along-extent>length:
                continue
            distance=side*(-ty*dx+tx*dy)
            uy=side*half_length*(-ty*c+tx*s)
            vy=side*half_width*(ty*s+tx*c)
            if distance-abs(uy)-abs(vy) >= -1e-6:
                continue
            if along-extent>=0 and along+extent<=length:
                return True
            if self.clipped_crosses(along,distance,ux,uy,vx,vy,length):
                return True
        return False


def footprint_crosses(pose,half_length,half_width,segments):
    if not segments: return False
    x,y,yaw=pose
    return YellowBoundary(segments).crosses(x,y,math.cos(yaw),math.sin(yaw),half_length,half_width)

"""Shared physical world specifications for rendering, scans and collision."""
from dataclasses import dataclass
import math
import numpy as np


def boundary_walls(world):
    """Four solid walls outside the mat, shared by sensors and collision."""
    spec = world.get('boundary_wall')
    if not spec:
        return []
    w, h = world['size']
    t, z = spec['thickness'], spec['height']
    return [
        {'name': 'boundary_wall_'+side, 'center': center, 'size': size,
         'color': spec['color']}
        for side, center, size in (
            ('west', (-t/2, h/2, z/2), (t, h+2*t, z)),
            ('east', (w+t/2, h/2, z/2), (t, h+2*t, z)),
            ('south', (w/2, -t/2, z/2), (w, t, z)),
            ('north', (w/2, h+t/2, z/2), (w, t, z)),
        )
    ]


def signs(world):
    dimensions=world.get('sign_geometry',{})
    diameter=dimensions.get('diameter',.09)
    total_height=dimensions.get('total_height',.20)
    defaults={'panel_width':diameter,'panel_height':diameter,
              'panel_center_height':total_height-diameter/2,
              'print_size':dimensions.get('print_diameter',.08),
              'post_width':dimensions.get('post_width',.01),
              'thickness':dimensions.get('thickness',.01)}
    result=[]
    if world.get('sign'):
        result.append({'name':'parking_sign','asset':'parking.png',**world['sign']})
    result.extend(world.get('signs',[]))
    fork=world.get('fork')
    if fork:
        result.append({'name':'fork_sign','asset':fork['assets'][fork['default']],'pose':fork['pose']})
    return [{**defaults,'shape':'triangle' if s['asset'] in ('speed-bump.png','pedestrian.png') else 'circle',**s}
            for s in result]


def expand_vehicle_layout(world):
    layout=world.get('avoidance_layout')
    if not layout:
        return
    width=(layout['x_max']-layout['x_min'])/layout['columns']
    if width<=0:
        raise ValueError('Avoidance layout requires x_max > x_min and positive columns')
    vehicles=[o for o in world['obstacles'] if not o['name'].startswith('avoid_')]
    index=0
    for row,columns in layout['occupied'].items():
        for column in columns:
            if not isinstance(column,int) or not 1<=column<=layout['columns']:
                raise ValueError('Avoidance column is outside the configured layout')
            index+=1
            vehicles.append({'name':f'avoid_{index:02d}','grid_row':row,'grid_column':column,
                             'pose':[layout['x_min']+(column-.5)*width,layout['row_y'][row],math.pi]})
    world['obstacles']=vehicles


@dataclass
class Cone:
    name: str
    x: float
    y: float
    height: float=.18
    radius: float=.038
    top_radius: float=.005
    base_size: float=.08
    base_height: float=.008

    def __post_init__(self):
        values=(self.x,self.y,self.height,self.radius,self.top_radius,self.base_size,self.base_height)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Cone dimensions and position must be finite')
        if not 0<self.base_height<self.height or not 0<self.top_radius<=self.radius or self.base_size<=0:
            raise ValueError('Cone requires positive radii and 0 < base height < total height')

    def radius_at(self,z):
        if not 0<=z<=self.height:
            return 0.
        if z<=self.base_height:
            return self.base_size/math.sqrt(2)
        t=(z-self.base_height)/(self.height-self.base_height)
        return self.radius*(1-t)+self.top_radius*t


def cones(world):
    result=[Cone(**c) for c in world.get('cones',[])]
    course=world.get('cone_course')
    if course:
        spec=course.get('cone',{})
        half_base=spec.get('base_size',.08)/2
        y0,y1=course['start_line_y']+half_base,course['end_line_y']-half_base
        if y1<=y0 or course['pairs']<2:
            raise ValueError('Cone course needs two stop lines and at least two pairs')
        margin=course['half_width'] if course.get('layout')!='double_s' else 0.
        y=np.linspace(y0-margin,y1+margin,max(1000,course['pairs']*80))
        x,dx=cone_course_shape(course,y)
        centre=np.column_stack((x,y))
        # Keep the paired rows horizontally separated for the sharp double S.
        # Perpendicular offsets would fold over themselves at its inner bends.
        normal=np.column_stack((np.ones_like(dx),np.zeros_like(dx)))
        if course.get('layout')!='double_s':
            normal[:,1]=-dx
            normal/=np.linalg.norm(normal,axis=1)[:,None]
        for side,name in ((-1,'left'),(1,'right')):
            edge=centre+side*course['half_width']*normal
            if np.any(np.diff(edge[:,1])<=0):
                raise ValueError('Cone course bends too tightly for its width; reduce amplitude or half_waves')
            endpoints=np.array([[np.interp(v,edge[:,1],edge[:,0]),v] for v in (y0,y1)])
            edge=np.vstack((endpoints[0],edge[(edge[:,1]>y0)&(edge[:,1]<y1)],endpoints[1]))
            # Equal arc spacing keeps both the bends and straight parts dense.
            length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(edge,axis=0),axis=1))]
            samples=np.linspace(0,length[-1],course['pairs'])
            positions=np.column_stack([np.interp(samples,length,edge[:,axis]) for axis in (0,1)])
            for i,p in enumerate(positions):
                result.append(Cone(f'cone_{name}_{i+1:02d}',*p,**spec))
    return result


def cone_course_shape(course,y):
    half_base=course.get('cone',{}).get('base_size',.08)/2
    y0,y1=course['start_line_y']+half_base,course['end_line_y']-half_base
    frequency=math.pi*course['half_waves']/(y1-y0)
    phase=(y-y0)*frequency+math.radians(course.get('phase_deg',0))
    return (course['center_x']+course['amplitude']*np.cos(phase),
            -course['amplitude']*frequency*np.sin(phase))


def obstacle_reference_path(min_radius=.462):
    """Feasible leftward lane change, used only for scene verification.

    Approach in the middle lane, shift down before column 8, then use the clear
    passage between lower/middle cars. This is not a mission node.
    """
    points=[(x,.94,math.pi) for x in np.linspace(8.55,8.30,80)]
    r=min_radius+.025
    angle=math.acos(1-.26/(2*r))
    centre=np.array([8.30,.94-r])
    for t in np.linspace(0,angle,150):
        points.append((centre[0]-r*math.sin(t),centre[1]+r*math.cos(t),math.pi+t))
    start=np.array(points[-1][:2])
    centre=start+r*np.array([-math.sin(angle),math.cos(angle)])
    for t in np.linspace(0,angle,150):
        yaw=math.pi+angle-t
        p=centre+r*np.array([-math.sin(yaw),math.cos(yaw)])
        points.append((*p,yaw))
    x=points[-1][0]
    points.extend((v,.68,math.pi) for v in np.linspace(x,4.45,300))
    return np.array(points)


def cone_reference_path(course=None):
    """Independent feasible verification route, never an autonomous controller.

    A wide double-S corridor permits a smoother vehicle path than its cone-row
    midpoint. Its own amplitude is explicit, so cone curvature is not mistaken
    for the LIMO's achievable steering radius.
    """
    course=course or {'start_line_y':1.86,'end_line_y':3.92,'center_x':.72,
                     'layout':'double_s','amplitude':.20,'reference_amplitude':.048,
                     'half_waves':4,'phase_deg':-90,'half_width':.32,'pairs':24}
    half_base=course.get('cone',{}).get('base_size',.08)/2
    y=np.linspace(course['start_line_y']+half_base,course['end_line_y']-half_base,600)
    reference={**course,'amplitude':course.get('reference_amplitude',course['amplitude'])}
    x,dx=cone_course_shape(reference,y)
    return np.column_stack((x,y,np.arctan2(np.ones_like(y),dx)))

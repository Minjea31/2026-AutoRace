"""Panda3D scene and actual perspective RGB/Z-buffer sensor rendering."""
import math
from pathlib import Path

import numpy as np
from PIL import Image
from panda3d.core import (
    AmbientLight, BitMask32, DirectionalLight, FrameBufferProperties,
    Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat,
    GeomVertexWriter, GraphicsOutput, LineSegs, PerspectiveLens, Point3,
    Texture, Vec3, loadPrcFileData,
)

from .config import share_directory
from .model import camera_mount, intrinsics
from .view_controls import OrbitView
from .world import boundary_walls, signs


def mesh(name, faces):
    data = GeomVertexData(name, GeomVertexFormat.getV3n3t2(), Geom.UHStatic)
    vertices, normals, texcoords = (GeomVertexWriter(data, field) for field in ('vertex', 'normal', 'texcoord'))
    triangles = GeomTriangles(Geom.UHStatic)
    index = 0
    for corners, normal in faces:
        for point, uv in zip(corners, ((0, 0), (1, 0), (1, 1), (0, 1))):
            vertices.addData3f(*point)
            normals.addData3f(*normal)
            texcoords.addData2f(*uv)
        triangles.addVertices(index, index+1, index+2)
        triangles.addVertices(index, index+2, index+3)
        index += 4
    geom = Geom(data)
    geom.addPrimitive(triangles)
    node = GeomNode(name)
    node.addGeom(geom)
    return node


def cube_node():
    return mesh('cube', [
        ([(-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)], (0,0,1)),
        ([(-.5,.5,-.5),(.5,.5,-.5),(.5,-.5,-.5),(-.5,-.5,-.5)], (0,0,-1)),
        ([(.5,-.5,-.5),(.5,.5,-.5),(.5,.5,.5),(.5,-.5,.5)], (1,0,0)),
        ([(-.5,.5,-.5),(-.5,-.5,-.5),(-.5,-.5,.5),(-.5,.5,.5)], (-1,0,0)),
        ([(.5,.5,-.5),(-.5,.5,-.5),(-.5,.5,.5),(.5,.5,.5)], (0,1,0)),
        ([(-.5,-.5,-.5),(.5,-.5,-.5),(.5,-.5,.5),(-.5,-.5,.5)], (0,-1,0)),
    ])


def plate_face(name,x,points,width,height,normal=1):
    """Convex plate face with continuous UV coordinates (circle or triangle)."""
    data=GeomVertexData(name,GeomVertexFormat.getV3n3t2(),Geom.UHStatic)
    vertex,vector,uv=(GeomVertexWriter(data,field) for field in ('vertex','normal','texcoord'))
    for y,z in [(0.,0.),*points]:
        vertex.addData3f(x,y,z)
        vector.addData3f(normal,0,0)
        uv.addData2f(y/width+.5,z/height+.5)
    triangles=GeomTriangles(Geom.UHStatic)
    for i in range(len(points)):
        a,b=i+1,(i+1)%len(points)+1
        triangles.addVertices(0,a,b) if normal>0 else triangles.addVertices(0,b,a)
    geom=Geom(data)
    geom.addPrimitive(triangles)
    node=GeomNode(name)
    node.addGeom(geom)
    return node


class Scene:
    def __init__(self, cfg, model, headless=False):
        loadPrcFileData('', '\n'.join([
            'audio-library-name null', 'sync-video false', 'textures-power-2 none',
            'win-size 1280 800', 'window-title AutoRace 2026 | LIMO Pro Simulator',
            'framebuffer-srgb false', 'notify-level warning',
            'model-cache-dir', 'texture-minfilter linear',
            *(['load-display p3headlessgl', 'window-type offscreen'] if headless else []),
        ]))
        from direct.showbase.ShowBase import ShowBase
        self.base = ShowBase(windowType='offscreen' if headless else None)
        if self.base.win is None:
            raise RuntimeError('Panda3D graphics initialization failed. Try --headless for EGL rendering.')
        self.base.disableMouse()
        self.base.setBackgroundColor(.075, .095, .12)
        self.cfg, self.model, self.headless = cfg, model, headless
        self.render = self.base.render
        self.cube = cube_node()
        self.view = 'parking'
        self.orbit = OrbitView()
        self.drag_mode = None
        self.last_mouse = None
        self.keys = dict.fromkeys(('w','arrow_up','arrow_down','a','d','arrow_left','arrow_right','space'),False)
        self.closed = self.paused = self.keyboard_enabled = False
        self.ros_domain = None
        self.scan_visible = cfg.get('display',{}).get('show_lidar',False)
        self.preview_visible = cfg.get('display',{}).get('show_camera',False)
        self.scan_node = None
        self.latest_scan = None
        self.body_mesh = self.base.loader.loadModel(str(share_directory()/'assets'/cfg['vehicle']['body_model']))
        self.wheel_mesh = self.base.loader.loadModel(str(share_directory()/'assets'/cfg['vehicle']['wheel_model']))
        ambient = self.render.attachNewNode(AmbientLight('ambient'))
        ambient.node().setColor((.65, .65, .65, 1))
        self.render.setLight(ambient)
        light = self.render.attachNewNode(DirectionalLight('sun'))
        light.node().setColor((.65, .65, .6, 1))
        light.setHpr(-25, -55, 0)
        self.render.setLight(light)
        self._ground()
        self.ego, self.wheels = self._vehicle(cfg['vehicle']['name'], (0, 0, 0), (1, 1, 1, 1))
        self.ego.hide(BitMask32.bit(1))  # physical sensors exclude ego surfaces
        self.base.cam.node().setCameraMask(BitMask32.bit(0))
        for obstacle in cfg['world']['obstacles']:
            self._vehicle(obstacle['name'], obstacle['pose'], (.82, .84, .87, 1))
        self.sign_faces={}
        self._sign()
        self.fork_direction=cfg['world'].get('fork',{}).get('default','left')
        self._cones()
        self._speed_bump()
        self._traffic_light()
        self.sensors = {stream: self._sensor(stream) for stream in ('color', 'depth')}
        self.base.camLens.setNearFar(.03, 100)
        self.base.camLens.setFov(58)
        if not headless:
            self._hud()
        self.update_pose()
        self.update_view()
        for _ in range(2):
            self.base.graphicsEngine.renderFrame()

    def box(self, parent, name, pos, size, color):
        node = parent.attachNewNode(self.cube.makeCopy())
        node.setName(name)
        node.setPos(*pos)
        node.setScale(*size)
        node.setColor(*color)
        return node

    def _ground(self):
        w, h = self.cfg['world']['size']
        node = self.render.attachNewNode(mesh('track', [([(0,0,0),(w,0,0),(w,h,0),(0,h,0)], (0,0,1))]))
        texture = self.base.loader.loadTexture(str(share_directory() / 'assets' / self.cfg['world']['texture']))
        texture.setMinfilter(Texture.FTLinearMipmapLinear)
        texture.setMagfilter(Texture.FTLinear)
        texture.setAnisotropicDegree(16)
        node.setTexture(texture)
        node.setLightOff()
        self.box(self.render, 'floor', (w/2,h/2,-.03), (w+4,h+4,.05), (.15,.17,.19,1))
        for wall in boundary_walls(self.cfg['world']):
            self.box(self.render, wall['name'], wall['center'], wall['size'], wall['color'])
        if not self.cfg.get('display',{}).get('show_parking_goal',False):
            return
        x0,x1,y0,y1 = self.cfg['world']['parking']['goal']
        lines = LineSegs('goal-debug')
        lines.setColor(.2,.95,.7,1)
        lines.setThickness(2)
        for i,(x,y) in enumerate(((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))):
            if i == 0:
                lines.moveTo(x,y,.004)
            else:
                lines.drawTo(x,y,.004)
        goal = self.render.attachNewNode(lines.create())
        goal.hide(BitMask32.bit(1))  # aids never appear in a sensor image

    def _vehicle(self, name, pose, color):
        v = self.cfg['vehicle']
        root = self.render.attachNewNode(name)
        root.setPos(pose[0],pose[1],0)
        root.setH(math.degrees(pose[2]))
        body = self.body_mesh.copyTo(root)
        body.setName('limo-pro-body')
        body.setScale(v['length']/.322,v['width']/.220,v['height']/.251)
        body.setColorScale(*color)
        wheels = []
        # Imported LIMO wheels retain their tread/rim and local Y axle.
        radius,width = v['wheel_radius'],v['wheel_width']
        for x in (-v['wheelbase']/2,v['wheelbase']/2):
            for y in (-v['wheel_track']/2,v['wheel_track']/2):
                pivot = root.attachNewNode('wheel-steering')
                pivot.setPos(x,y,radius)
                tire = self.wheel_mesh.copyTo(pivot)
                tire.setScale(radius/.045,width/.045,radius/.045)
                wheels.append((pivot,tire,x,y))
        return root,wheels

    def _sign(self):
        for s in signs(self.cfg['world']):
            root=self.render.attachNewNode(s['name'])
            root.setPos(s['pose'][0],s['pose'][1],0)
            root.setH(math.degrees(s['pose'][2]))
            height=s['panel_center_height']-s['panel_height']/2
            self.box(root,'post',(0,0,height/2),(s['post_width'],s['post_width'],height),(.65,.65,.65,1))
            width,height,thickness=s['panel_width'],s['panel_height'],s['thickness']
            if s['shape']=='circle':
                points=[(width/2*math.cos(a),height/2*math.sin(a)) for a in np.linspace(0,math.tau,64,endpoint=False)]
            else:
                points=[(-width/2,-height/2),(width/2,-height/2),(0.,height/2)]
            plate=root.attachNewNode('plate')
            plate.setZ(s['panel_center_height'])
            plate.attachNewNode(plate_face('plate-front',thickness/2,points,width,height)).setColor(.92,.92,.88,1)
            plate.attachNewNode(plate_face('plate-back',-thickness/2,points,width,height,-1)).setColor(.72,.74,.72,1)
            edges=[]
            for i,(y0,z0) in enumerate(points):
                y1,z1=points[(i+1)%len(points)]
                normal=Vec3(0,z1-z0,y0-y1)
                normal.normalize()
                edges.append(([(-thickness/2,y0,z0),(-thickness/2,y1,z1),
                               (thickness/2,y1,z1),(thickness/2,y0,z0)],tuple(normal)))
            plate.attachNewNode(mesh('plate-rim',edges)).setColor(.66,.68,.65,1)
            printed=[(y*s['print_size']/width,z*s['print_size']/height) for y,z in points]
            face=plate.attachNewNode(plate_face(s['name']+'-face',thickness/2+.0002,
                                     printed,s['print_size'],s['print_size']))
            face.setTexture(self._sign_texture(s['asset']))
            face.setLightOff()
            self.sign_faces[s['name']]=face

    def _sign_texture(self,asset):
        name=Path(asset).stem+'-print.png'
        texture=self.base.loader.loadTexture(str(share_directory()/'assets'/name))
        texture.setMinfilter(Texture.FTLinearMipmapLinear)
        texture.setAnisotropicDegree(8)
        return texture

    def set_fork(self,direction):
        if direction not in ('left','right'):
            raise ValueError('Fork direction must be left or right')
        self.fork_direction=direction
        if 'fork_sign' in self.sign_faces:
            name=self.cfg['world']['fork']['assets'][direction]
            self.sign_faces['fork_sign'].setTexture(self._sign_texture(name))

    def _cones(self):
        for cone in self.model.cones:
            root=self.render.attachNewNode(cone.name)
            root.setPos(cone.x,cone.y,0)
            self.box(root,'base',(0,0,cone.base_height/2),
                     (cone.base_size,cone.base_size,cone.base_height),(1,.28,.02,1))
            for z0,z1,colour in ((cone.base_height,.075,(1,.25,.015,1)),
                                 (.075,.115,(.96,.96,.94,1)),(.115,cone.height,(1,.25,.015,1))):
                r0,r1=cone.radius_at(z0+.000001),cone.radius_at(z1)
                faces=[]
                for i in range(24):
                    a,b=math.tau*i/24,math.tau*(i+1)/24
                    normal=Vec3(math.cos((a+b)/2),math.sin((a+b)/2),(r0-r1)/(z1-z0))
                    normal.normalize()
                    faces.append(([(r0*math.cos(a),r0*math.sin(a),z0),
                                   (r0*math.cos(b),r0*math.sin(b),z0),
                                   (r1*math.cos(b),r1*math.sin(b),z1),
                                   (r1*math.cos(a),r1*math.sin(a),z1)],tuple(normal)))
                node=root.attachNewNode(mesh(cone.name+'-frustum',faces))
                node.setColor(*colour)

    def _speed_bump(self):
        s=self.cfg['world'].get('speed_bump')
        if not s:
            return
        root=self.render.attachNewNode('speed-bump')
        root.setPos(s['pose'][0],s['pose'][1],0)
        root.setH(math.degrees(s['pose'][2]))
        for i in range(10):
            colour=(1,.80,.05,1) if i%2==0 else (.035,.035,.035,1)
            self.box(root,'bump-section',(0,-s['width']/2+(i+.5)*s['width']/10,s['height']/2),
                     (s['length'],s['width']/10,s['height']),colour)

    def _traffic_light(self):
        self.traffic_bulbs={}
        self.traffic_state='red'
        for key,thickness in (('traffic_light',.035),('advertisement',.02)):
            s=self.cfg['world'].get(key)
            if not s:
                continue
            root=self.render.attachNewNode(key)
            root.setPos(s['pose'][0],s['pose'][1],0)
            root.setH(math.degrees(s['pose'][2]))
            bottom=s['center_height']-s['height']/2
            self.box(root,'post',(0,0,bottom/2),(.018,.018,bottom),(.5,.5,.5,1))
            self.box(root,'housing',(0,0,s['center_height']),(thickness,s['width'],s['height']),(.025,.03,.035,1))
            if key=='traffic_light':
                for index,state in enumerate(('red','yellow','green')):
                    z=s['center_height']+s['height']*(1-index)/3
                    radius=s['width']*.27
                    faces=[]
                    x=thickness/2+.0005
                    for i in range(32):
                        a,b=math.tau*i/32,math.tau*(i+1)/32
                        faces.append(([(x,0,z),(x,radius*math.cos(a),z+radius*math.sin(a)),
                                       (x,radius*math.cos(b),z+radius*math.sin(b)),(x,0,z)],(1,0,0)))
                    bulb=root.attachNewNode(mesh('bulb-'+state,faces))
                    bulb.setLightOff()
                    bulb.setTwoSided(True)
                    self.traffic_bulbs[state]=bulb
            else:
                for index,colour in enumerate(((.05,.12,.5,1),(.8,.08,.07,1),(.1,.6,.3,1))):
                    self.box(root,'advert-colour',(thickness/2+.001,0,s['center_height']+(index-1)*s['height']/3),
                             (.001,s['width']*.92,s['height']/3),colour).setLightOff()
                from panda3d.core import TextNode
                text=TextNode('advert-text')
                text.setText('AUTO\nSALE')
                text.setTextColor(1,1,1,1)
                label=root.attachNewNode(text)
                label.setH(90)
                label.setScale(.026)
                label.setPos(thickness/2+.002,-s['width']*.4,s['center_height']+.015)
                label.setLightOff()
        self.set_traffic(self.cfg['world'].get('traffic_light',{}).get('default','red'))

    def set_traffic(self,state):
        if state not in ('red','yellow','green'):
            raise ValueError('Traffic light must be red, yellow or green')
        self.traffic_state=state
        colours={'red':(1,.025,.015,1),'yellow':(1,.75,.015,1),'green':(.015,1,.09,1)}
        for key,node in self.traffic_bulbs.items():
            node.setColor(*(colours[key] if key==state else (.045,.055,.045,1)))

    def _sensor(self, stream):
        c = self.cfg['camera'][stream]
        w,h = c['width'],c['height']
        fb = FrameBufferProperties()
        fb.setRgbColor(True)
        fb.setRgbaBits(8,8,8,0)
        fb.setDepthBits(32)
        color = Texture(stream+'-rgb')
        buffer = self.base.win.makeTextureBuffer(stream+'-buffer',w,h,color,True,fb)
        if buffer is None:
            raise RuntimeError('Could not allocate sensor render buffer')
        buffer.setClearColor((.12,.16,.21,1))
        buffer.setSort(-10)
        lens = PerspectiveLens()
        fx,fy,cx,cy = intrinsics(c)
        lens.setFilmSize(w/fx,h/fy)
        lens.setFocalLength(1)
        lens.setFilmOffset((w/2-cx-.5)/fx,(cy+.5-h/2)/fy)
        lens.setNearFar(self.cfg['camera']['near'],self.cfg['camera']['far'])
        camera = self.base.makeCamera(buffer,lens=lens,scene=self.render)
        camera.node().setCameraMask(BitMask32.bit(1))
        camera.reparentTo(self.ego)
        p,r = camera_mount(self.cfg,stream)
        p = p + [0,0,self.cfg['vehicle']['base_link_height']]
        camera.setPos(*p)
        camera.lookAt(self.ego,Point3(*(p+r[:,0])),Vec3(*r[:,2]))
        depth = None
        if stream == 'depth':
            depth = Texture('depth-z')
            depth.setup2dTexture(w,h,Texture.TFloat,Texture.FDepthComponent32)
            buffer.addRenderTexture(depth,GraphicsOutput.RTMCopyRam,GraphicsOutput.RTPDepth)
        return {'buffer':buffer,'camera':camera,'rgb':color,'z':depth}

    def _hud(self):
        from direct.gui.OnscreenText import OnscreenText
        from direct.gui.OnscreenImage import OnscreenImage
        from panda3d.core import TextNode
        self.title = OnscreenText(text='LIMO PRO / AUTORACE LAB',pos=(-1.52,.91),scale=.055,
                                  align=TextNode.ALeft,fg=(.5,.9,1,1),mayChange=False)
        self.hud = OnscreenText(text='',pos=(-1.52,.80),scale=.037,align=TextNode.ALeft,
                                fg=(.94,.97,1,1),mayChange=True)
        self.help = OnscreenText(text='K: keyboard   W/UP: forward   DOWN: reverse   A/D: steer   SPACE: stop\n'
            'Left drag: pan   Right drag: orbit   Wheel: zoom   V: camera   S: laser\n'
            'L/R: fork left/right   T/G/Y: light red/green/yellow   F5: reset\n'
            '1: parking   2: whole track   3: follow   4: camera   P: pause   ESC: quit',
            pos=(-1.52,-.85),scale=.029,align=TextNode.ALeft,fg=(.8,.86,.92,1))
        self.preview = OnscreenImage(image=self.sensors['color']['rgb'],pos=(1.19,0,.66),
            scale=(.36,1,.36*self.cfg['camera']['color']['height']/self.cfg['camera']['color']['width']))
        if not self.preview_visible:
            self.preview.hide()
        for key in self.keys:
            self.base.accept(key,self._key,[key,True])
            self.base.accept(key+'-up',self._key,[key,False])
        self.base.accept('escape',self._close)
        self.base.accept('k',self._keyboard)
        self.base.accept('p',self._pause)
        self.base.accept('f5',self.reset)
        self.base.accept('l',self.set_fork,['left'])
        self.base.accept('r',self.set_fork,['right'])
        self.base.accept('s',self._scan_toggle)
        self.base.accept('t',self.set_traffic,['red'])
        self.base.accept('g',self.set_traffic,['green'])
        self.base.accept('y',self.set_traffic,['yellow'])
        self.base.accept('v',self._preview_toggle)
        self.base.accept('mouse1',self._begin_drag,['pan'])
        self.base.accept('mouse1-up',self._end_drag,['pan'])
        self.base.accept('mouse3',self._begin_drag,['orbit'])
        self.base.accept('mouse3-up',self._end_drag,['orbit'])
        self.base.accept('wheel_up',self._zoom,[1])
        self.base.accept('wheel_down',self._zoom,[-1])
        for key,view in zip(('1','2','3','4'),('parking','overview','follow','camera')):
            self.base.accept(key,self.set_view,[view])

    def _key(self,key,value):
        self.keys[key] = value

    def _close(self):
        self.closed = True

    def _keyboard(self):
        self.keyboard_enabled = not self.keyboard_enabled
        self.model.command(0,0)

    def _pause(self):
        self.paused = not self.paused

    def _scan_toggle(self):
        self.scan_visible = not self.scan_visible
        if self.latest_scan is not None:
            self.update_scan(self.latest_scan)

    def _preview_toggle(self):
        self.preview_visible = not self.preview_visible
        self.preview.show() if self.preview_visible else self.preview.hide()

    def _free_view(self):
        if self.view == 'free':
            return
        eye = np.array(tuple(self.base.camera.getPos(self.render)))
        if self.view in ('parking','overview'):
            target = self.orbit.target
        elif self.view == 'follow':
            m = self.model
            target = (m.x+.25*math.cos(m.yaw),m.y+.25*math.sin(m.yaw),.12)
        else:
            forward = np.array(tuple(self.base.camera.getQuat(self.render).getForward()))
            target = eye+forward*1.2
        self.orbit.set_pose(eye,target)
        self.view = 'free'

    def _begin_drag(self,mode):
        self.update_view()
        self._free_view()
        self.drag_mode = mode
        watcher = self.base.mouseWatcherNode
        self.last_mouse = tuple(watcher.getMouse()) if watcher is not None and watcher.hasMouse() else None

    def _end_drag(self,mode):
        if self.drag_mode == mode:
            self.drag_mode = None
            self.last_mouse = None

    def _drag_mouse(self,dx,dy):
        if self.drag_mode == 'pan':
            self.orbit.pan(dx,dy,self.base.camLens.getHfov(),self.base.camLens.getVfov())
        elif self.drag_mode == 'orbit':
            self.orbit.rotate(dx,dy)

    def _poll_mouse(self):
        watcher = self.base.mouseWatcherNode
        if self.drag_mode is None or watcher is None or not watcher.hasMouse():
            self.last_mouse = None
            return
        current = tuple(watcher.getMouse())
        if self.last_mouse is not None:
            self._drag_mouse(current[0]-self.last_mouse[0],current[1]-self.last_mouse[1])
        self.last_mouse = current

    def _zoom(self,steps):
        self.update_view()
        self._free_view()
        self.orbit.zoom(steps)

    def reset(self):
        self.model.reset()

    def set_view(self,view):
        self.view = view
        self.drag_mode = self.last_mouse = None
        if view == 'parking':
            self.orbit.set_pose((6.9,2.4,3.2),(6.65,4.45,0))
        elif view == 'overview':
            # Avoid the exactly vertical lookAt singularity, keeping pan aligned
            # with the screen's horizontal/vertical directions from above.
            self.orbit.set_pose((5,2.99,10.5),(5,3,0))

    def keyboard_command(self):
        if not self.keyboard_enabled:
            return
        speed=.25*(int(self.keys['w'] or self.keys['arrow_up'])-int(self.keys['arrow_down']))
        steer=self.model.max_steering*(int(self.keys['a'] or self.keys['arrow_left'])
                                      -int(self.keys['d'] or self.keys['arrow_right']))
        if self.keys['space']:
            speed = 0
        self.model.command(speed,speed*math.tan(steer)/self.cfg['vehicle']['wheelbase'])

    def update_pose(self):
        m,v = self.model,self.cfg['vehicle']
        self.ego.setPos(m.x,m.y,0)
        self.ego.setH(math.degrees(m.yaw))
        for pivot,tire,x,y in self.wheels:
            angle = 0
            if x > 0 and abs(m.steering)>1e-8:
                radius = v['wheelbase']/math.tan(m.steering)
                angle = math.atan(v['wheelbase']/(radius-y))
            pivot.setH(math.degrees(angle))
            tire.setR(math.degrees(m.wheel_travel/v['wheel_radius']))

    def update_view(self):
        m = self.model
        if self.view in ('parking','overview','free'):
            self.base.camera.setPos(*self.orbit.eye())
            self.base.camera.lookAt(Point3(*self.orbit.target))
        elif self.view == 'follow':
            self.base.camera.setPos(m.x-1.1*math.cos(m.yaw),m.y-1.1*math.sin(m.yaw),.9)
            self.base.camera.lookAt(m.x+.25*math.cos(m.yaw),m.y+.25*math.sin(m.yaw),.12)
        elif self.view == 'camera':
            self.base.camera.setMat(self.sensors['color']['camera'].getMat(self.render))

    def update_scan(self,ranges):
        self.latest_scan = np.asarray(ranges).copy()
        if self.headless:
            return
        if not self.scan_visible:
            if self.scan_node:
                self.scan_node.hide()
            return
        if self.scan_node:
            self.scan_node.removeNode()
        lines = LineSegs('lidar-debug')
        lines.setThickness(1)
        lines.setColor(1,.35,.1,.75)
        cfg = self.cfg['lidar']
        x,y,z = cfg['xyz']
        z += self.cfg['vehicle']['base_link_height']
        for angle,distance in zip(self.model.angles,ranges):
            if distance > 0 and math.isfinite(distance):
                lines.moveTo(x,y,z)
                lines.drawTo(x+distance*math.cos(angle+cfg['yaw']),y+distance*math.sin(angle+cfg['yaw']),z)
        self.scan_node = self.ego.attachNewNode(lines.create())
        self.scan_node.hide(BitMask32.bit(1))

    def render_frame(self,streams=()):
        # Only refresh the camera buffers at their configured sensor rates.
        for stream,data in self.sensors.items():
            data['buffer'].setActive(stream in streams)
        if self.headless:
            self.base.win.setActive(bool(streams))
        self.update_pose()
        if not self.headless:
            self._poll_mouse()
        self.update_view()
        if not self.headless:
            s = self.model.status()
            ros_state = f'ROS domain {self.ros_domain}' if self.ros_domain is not None else 'ROS OFF'
            control = 'KEYBOARD' if self.keyboard_enabled else ('ROS /cmd_vel' if self.ros_domain is not None else 'IDLE (K: keyboard)')
            self.hud.setText(f"t={s['time']:.2f}s  speed={s['speed']:+.2f} m/s  steer={math.degrees(s['steering_angle']):+.1f} deg\n"
                f"collisions {s['collisions']}  {'CONTACT' if s['collision'] else 'CLEAR'}\n"
                f"{'PAUSED' if self.paused else 'RUNNING'}  "
                f"control: {control}\n"
                f"{ros_state}\n"
                f"fork: {self.fork_direction.upper()}   traffic: {self.traffic_state.upper()}")
        self.base.taskMgr.step()

    def color_image(self):
        c = self.cfg['camera']['color']
        data = self.sensors['color']['rgb'].getRamImageAs('RGB')
        if not data:
            raise RuntimeError('RGB render-to-RAM returned no pixels')
        return np.frombuffer(data,dtype=np.uint8).reshape(c['height'],c['width'],3)[::-1].copy()

    def depth_image(self):
        c = self.cfg['camera']['depth']
        texture = self.sensors['depth']['z']
        if not texture.hasRamImage() or texture.getComponentType() != Texture.TFloat:
            raise RuntimeError('Graphics backend did not provide float32 depth texture')
        z = np.frombuffer(texture.getRamImage(),dtype=np.float32).reshape(c['height'],c['width'])[::-1]
        near,far = self.cfg['camera']['near'],self.cfg['camera']['far']
        metric = near*far/(far-z*(far-near))
        valid = (z < 1-1e-7) & (metric >= c['range_min']) & (metric <= c['range_max'])
        return np.where(valid,np.rint(metric*1000),0).astype('<u2')

    def snapshot(self,path):
        target = Path(path)
        target.mkdir(parents=True,exist_ok=True)
        self.base.win.setActive(True)
        for data in self.sensors.values():
            data['buffer'].setActive(True)
        self.base.graphicsEngine.renderFrame()
        self.base.win.getScreenshot().write(str(target/'scene.png'))
        Image.fromarray(self.color_image()).save(target/'color.png')
        Image.fromarray(self.depth_image()).save(target/'depth_mm.png')

    def close(self):
        self.base.destroy()

"""Offscreen geometric checks: actual RGB image, Z-buffer units and optical axes."""
import numpy as np
import pytest

from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation


def test_low_traffic_light_dimensions_and_sensor_colour_switch():
    from limo_parking_sim.scene import Scene
    cfg = load_config()
    cfg['simulation']['spawn'] = cfg['world']['spawns']['signal']
    model = Simulation(cfg)
    scene = Scene(cfg, model, headless=True)
    try:
        root = scene.render.find('**/traffic_light')
        lo, hi = root.getTightBounds(root)
        assert hi.y-lo.y == pytest.approx(.12, abs=1e-6)
        assert lo.z == pytest.approx(0, abs=1e-6)
        assert hi.z == pytest.approx(.255, abs=1e-6)
        lo, hi = root.find('housing').getTightBounds(root)
        assert lo.z == pytest.approx(.05, abs=1e-6)
        assert hi.z-lo.z == pytest.approx(.205, abs=1e-6)
        # Sensor/collision boxes must use the same low stand and brackets.
        for box in (b for b in model.obstacles if b.name.startswith('traffic_light_')):
            lo, hi = root.find(box.name.removeprefix('traffic_light_')).getTightBounds(scene.render)
            assert (lo.z, hi.z) == pytest.approx((box.zmin, box.zmax), abs=1e-6)
            assert (lo.x, lo.y) == pytest.approx(box.corners().min(axis=0), abs=1e-6)
            assert (hi.x, hi.y) == pytest.approx(box.corners().max(axis=0), abs=1e-6)
        colours = []
        depths = []
        for state in ('red', 'yellow', 'green'):
            scene.set_traffic(state)
            assert scene.traffic_state == state
            scene.render_frame({'color', 'depth'})
            colours.append(scene.color_image())
            depths.append(scene.depth_image())
        for before, after in zip(colours, colours[1:]):
            assert np.count_nonzero(np.max(np.abs(before.astype(float)-after), axis=2) > 32) > 20
        assert np.array_equal(depths[0], depths[1])
        assert np.array_equal(depths[0], depths[2])
        # At sensor height the lowered housing, rather than the old tall post,
        # must now intersect the centre lidar ray.
        model.x, model.y, model.yaw = 2.45, 4.49, 0.
        scan = model.scan()
        index = np.argmin(np.abs(model.angles))
        assert scan[index] == pytest.approx(3.17-.035/2-2.45-.103, abs=.002)
    finally:
        scene.close()


def test_rendered_depth_rgb_orientation_and_camera_frame():
    from limo_parking_sim.scene import Scene
    cfg = load_config()
    cfg['simulation']['spawn'] = [3.,3.,0.]
    model = Simulation(cfg)
    scene = Scene(cfg,model,headless=True)
    try:
        # Known front surface at x=4.0; optical centre is x=3.133.
        wall = scene.box(scene.render,'reference-wall',(4.05,2.95,1),(.1,2,2),(.9,.03,.03,1))
        wall.setLightOff()
        # Local +Y is image-left and +Z is image-up in ROS optical coordinates.
        marker = scene.box(scene.render,'orientation-marker',(3.997,3.15,.36),(.002,.06,.06),(.01,.01,1,1))
        marker.setLightOff()
        for _ in range(2):
            scene.render_frame({'color','depth'})
        color,depth = scene.color_image(),scene.depth_image()
        assert color.shape == (480,640,3)
        assert depth.shape == (400,640)
        assert depth.dtype == np.dtype('<u2')
        assert np.median(depth[170:230,280:360]) == pytest.approx(867,abs=2)
        assert color[240,320,0] > 180 and color[240,320,1] < 50
        blue = (color[:,:,2] > 200) & (color[:,:,0] < 50)
        rows,cols = np.nonzero(blue)
        assert rows.size > 100
        assert rows.mean() < 240 and cols.mean() < 320
        assert color.flags['C_CONTIGUOUS']
        # Camera +Y (Panda forward) must coincide with ROS link +X.
        from panda3d.core import Vec3
        forward = scene.render.getRelativeVector(scene.sensors['color']['camera'],Vec3(0,1,0))
        assert tuple(forward) == pytest.approx((1,0,0),abs=1e-6)
        # Build the same HUD and dispatch the exact GUI events in an offscreen
        # window. This checks bindings without taking over the user's desktop.
        scene._hud()
        scene.headless = False
        scene.base.messenger.send('k')
        assert scene.keyboard_enabled
        scene.base.messenger.send('w')
        scene.base.messenger.send('a')
        for _ in range(60):
            scene.keyboard_command()
            model.step(1/120)
        scene.render_frame({'color','depth'})
        assert model.x > 3.05 and model.yaw > .05
        assert scene.ego.getX() == pytest.approx(model.x,abs=1e-6)
        assert scene.ego.getY() == pytest.approx(model.y,abs=1e-6)
        assert scene.ego.getH() == pytest.approx(np.degrees(model.yaw),abs=1e-5)
        assert any(abs(pivot.getH())>.1 for pivot,_,x,_ in scene.wheels if x>0)
        assert np.mean(np.abs(scene.color_image().astype(float)-color)) > 1
        scene.base.messenger.send('w-up')
        scene.base.messenger.send('a-up')
        scene.base.messenger.send('k')
        assert not scene.keyboard_enabled
        scene.base.messenger.send('p')
        assert scene.paused
        scene.base.messenger.send('p')
        assert not scene.paused
        scene.base.messenger.send('2')
        assert scene.view == 'overview'
        scene.base.messenger.send('f5')
        assert model.x == 3 and model.y == 3 and model.speed == 0
        # Observer controls must not alter physical RGB/depth cameras, lidar or
        # vehicle pose. Hidden overlays must not stop sensor rendering either.
        assert not scene.scan_visible and not scene.preview_visible
        assert scene.preview.isHidden()
        scene.update_scan(model.scan())
        assert scene.scan_node is None
        scene.base.messenger.send('i')
        assert scene.scan_visible and scene.scan_node is not None
        scene.base.messenger.send('i')
        assert not scene.scan_visible and scene.scan_node.isHidden()
        scene.base.messenger.send('v')
        assert scene.preview_visible and not scene.preview.isHidden()
        scene.base.messenger.send('v')
        assert not scene.preview_visible and scene.preview.isHidden()
        scene.set_view('parking')
        scene.render_frame({'color','depth'})
        sensor_matrix = scene.sensors['color']['camera'].getMat(scene.render)
        sensor_image = scene.color_image().copy()
        scan = model.scan().copy()
        pose = (model.x,model.y,model.yaw)
        target,distance = scene.orbit.target.copy(),scene.orbit.distance
        scene.base.messenger.send('mouse1')
        scene._drag_mouse(.15,-.1)
        scene.base.messenger.send('mouse1-up')
        assert scene.view=='free' and scene.drag_mode is None
        assert np.linalg.norm(scene.orbit.target-target)>.1
        assert scene.orbit.distance==pytest.approx(distance)
        scene.render_frame({'color','depth'})
        target,angle = scene.orbit.target.copy(),scene.orbit.azimuth
        scene.base.messenger.send('mouse3')
        scene._drag_mouse(.2,.1)
        scene.base.messenger.send('mouse3-up')
        scene.base.messenger.send('wheel_up')
        scene.render_frame({'color','depth'})
        assert scene.orbit.azimuth!=angle
        assert scene.orbit.target==pytest.approx(target)
        assert scene.orbit.distance<distance
        assert scene.sensors['color']['camera'].getMat(scene.render)==sensor_matrix
        assert np.array_equal(scene.color_image(),sensor_image)
        assert np.array_equal(model.scan(),scan)
        assert (model.x,model.y,model.yaw)==pose
        # Observer debug rays are deliberately outside the vehicle envelope.
        scene.scan_node.removeNode()
        scene.scan_node = None
        lo,hi = scene.ego.getTightBounds()
        assert tuple(hi-lo)==pytest.approx((.322,.220,.251),abs=.004)
        assert scene.render.find('**/goal-debug').isEmpty()
        sign=scene.render.find('**/slow_sign')
        lo,hi=sign.getTightBounds(scene.render)
        assert hi.z==pytest.approx(.20,abs=1e-6)
        assert lo.z==pytest.approx(0,abs=1e-6)
        lo,hi=sign.find('**/plate').getTightBounds(scene.render)
        assert hi.z-lo.z==pytest.approx(.09,abs=1e-6)
        assert hi.y-lo.y==pytest.approx(.09,abs=1e-6)
        # R/L must replace the real fork texture without resetting/moving ego;
        # I toggles laser overlays; S and Down both reverse without toggling it.
        model.x,model.y,model.yaw=cfg['world']['spawns']['fork']
        scene.render_frame({'color','depth'})
        left=scene.color_image().copy()
        pose=(model.x,model.y,model.yaw)
        scene.base.messenger.send('r')
        scene.render_frame({'color','depth'})
        right=scene.color_image().copy()
        assert scene.fork_direction=='right'
        assert np.count_nonzero(np.max(np.abs(right.astype(float)-left),axis=2)>32)>40
        assert (model.x,model.y,model.yaw)==pose and not scene.scan_visible
        scene.base.messenger.send('l')
        scene.render_frame({'color','depth'})
        assert scene.fork_direction=='left'
        assert np.array_equal(scene.color_image(),left)
        scene.base.messenger.send('g')
        assert scene.traffic_state=='green'
        scene.base.messenger.send('t')
        assert scene.traffic_state=='red'
        scene.base.messenger.send('y')
        assert scene.traffic_state=='yellow'
        scene.base.messenger.send('f5')
        scene.base.messenger.send('k')
        for key in ('s', 'arrow_down'):
            scene.base.messenger.send(key)
            for _ in range(30):
                scene.keyboard_command()
                model.step(1/120)
            assert model.speed<0 and model.cmd_speed == -1.0
            assert not scene.scan_visible
            scene.base.messenger.send(key+'-up')
        scene.base.messenger.send('k')
        # The real perimeter is black, 30 cm tall, and visible to RGB/depth.
        for side in ('west', 'east', 'south', 'north'):
            wall = scene.render.find('**/boundary_wall_'+side)
            lo,hi = wall.getTightBounds(scene.render)
            assert lo.z == pytest.approx(0, abs=1e-6)
            assert hi.z == pytest.approx(.30, abs=1e-6)
            assert tuple(wall.getColor()) == pytest.approx((.015,.015,.015,1), abs=1/1024)
        model.x,model.y,model.yaw = 9.4,3.,0.
        for _ in range(2):
            scene.render_frame({'color','depth'})
        assert np.median(scene.depth_image()[190:210,310:330]) == pytest.approx(467, abs=2)
        assert np.max(scene.color_image()[230:250,310:330]) < 20
    finally:
        scene.close()


def test_live_cone_and_parking_placement_update_rendering(tmp_path):
    from limo_parking_sim.scene import Scene
    from limo_parking_sim.obstacle_editor import Placement
    cfg = load_config()
    cfg['simulation']['spawn'] = [3., 3., 0.]
    cfg['world']['obstacles'] = []
    cfg['world']['cones'] = []
    cfg['world']['cone_course']['enabled'] = False
    model = Simulation(cfg)
    scene = Scene(cfg, model, headless=True)
    placement = Placement(cfg, model, scene.refresh_obstacles, path=tmp_path/'layout.yaml')
    try:
        scene.render_frame({'color', 'depth'})
        color, depth, scan = scene.color_image(), scene.depth_image(), model.scan()
        placement.set_kind('cone')
        assert placement.click(3.65, 3.)
        name = placement.cones[-1]['name']
        scene.render_frame({'color', 'depth'})
        assert not scene.render.find('**/'+name).isEmpty()
        assert np.count_nonzero(scene.color_image() != color) > 100
        assert np.count_nonzero(scene.depth_image() != depth) > 100
        assert model.scan()[len(model.angles)//2] < scan[len(model.angles)//2]
        assert placement.click(3.65, 3., button=3)
        scene.render_frame({'color', 'depth'})
        assert scene.render.find('**/'+name).isEmpty()
        assert np.array_equal(scene.color_image(), color)
        assert np.array_equal(scene.depth_image(), depth)
        placement.set_kind('vehicle')
        assert placement.click(6.88, 4.27)
        car_name = placement.obstacles[-1]['name']
        assert not scene.render.find('**/'+car_name).isEmpty()
        pose = model.x, model.y, model.yaw
        camera = scene.sensors['color']['camera'].getMat(scene.render)
        for region in ('cone', 'parking', 'obstacle'):
            scene.focus_editor_region(region)
            assert (model.x, model.y, model.yaw) == pose
            assert scene.sensors['color']['camera'].getMat(scene.render) == camera
        placement.undo()
        assert scene.render.find('**/'+car_name).isEmpty()
    finally:
        scene.close()

import math
import unittest
from types import SimpleNamespace as N
from mission_core import MarkerStore,Telemetry,Leases,marker_geometry,render_markers,IDENTITY

def point(x=0,y=0,z=0):return N(x=float(x),y=float(y),z=float(z))
def marker(id=1,ns='test',action=0,life=0,frame='base_link',locked=False):
    return N(id=id,ns=ns,action=action,type=4,frame_locked=locked,lifetime=N(sec=int(life),nanosec=int((life%1)*1e9)),header=N(frame_id=frame,stamp=N(sec=10,nanosec=0)),pose=N(position=point(),orientation=N(x=0,y=0,z=0,w=1)),points=[point(1),point(2)],scale=point(.02,.02,.1),color=N(r=1.,g=.5,b=.25,a=.8),colors=[],text='hello')

class CoreTests(unittest.TestCase):
    def setUp(self):self.now=100.;self.clock=lambda:self.now
    def test_actions_are_never_throttled(self):
        s=MarkerStore(self.clock);s.receive('/m',[marker()]);s.receive('/m',[marker(action=3)]);self.assertEqual(s.records('/m'),[])
        s.receive('/m',[marker(1),marker(2),marker(1,'other')]);s.receive('/m',[marker(1,action=2)])
        self.assertEqual({(x['message'].ns,x['message'].id) for x in s.records('/m')},{('test',2),('other',1)})
        s.receive('/m',[marker(action=3),marker(8)]);self.assertEqual([r['message'].id for r in s.records('/m')],[8])
    def test_lifetime_and_replacement(self):
        s=MarkerStore(self.clock);s.receive('/m',[marker(life=.5)]);self.now+=.4;s.receive('/m',[marker(life=.5)]);self.now+=.4;self.assertEqual(len(s.records('/m')),1);self.now+=.11;self.assertEqual(s.records('/m'),[])
    def test_pose_rotation_is_applied_to_marker_points(self):
        m=marker();m.pose.position=point(2,3,4);m.pose.orientation=N(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))
        g=marker_geometry(m);self.assertAlmostEqual(g['points'][0][0],2);self.assertAlmostEqual(g['points'][0][1],4);self.assertAlmostEqual(g['points'][0][2],4);self.assertEqual(g['color'],[1,.5,.25,.8])
    def test_freeze_at_stamp_then_follow_current_ego(self):
        s=MarkerStore(self.clock);s.receive('/m',[marker()]);ego=[10.]
        def tf(target,source,stamp):
            if target==source:return IDENTITY
            if (target,source)==('odom','base_link'):return ([10 if stamp else ego[0],0,0],[0,0,0,1])
            if (target,source)==('base_link','odom'):return ([-ego[0],0,0],[0,0,0,1])
            raise ValueError('missing TF')
        layers,errors=render_markers(s,['/m'],'base_link',tf);self.assertFalse(errors);self.assertAlmostEqual(layers[0]['points'][0][0],1)
        ego[0]=12;layers,_=render_markers(s,['/m'],'base_link',tf);self.assertAlmostEqual(layers[0]['points'][0][0],-1)
    def test_frame_locked_uses_latest_transform(self):
        s=MarkerStore(self.clock);s.receive('/m',[marker(locked=True)]);calls=[]
        def tf(target,source,stamp):
            calls.append((target,source,stamp));return IDENTITY
        render_markers(s,['/m'],'odom',tf);self.assertTrue(all(stamp is None for _,_,stamp in calls))
    def test_missing_tf_hides_instead_of_drawing_in_wrong_frame(self):
        s=MarkerStore(self.clock);s.receive('/m',[marker(frame='missing')])
        def tf(*args):raise ValueError('no transform')
        layers,errors=render_markers(s,['/m'],'base_link',tf);self.assertFalse(layers);self.assertTrue(errors)
    def test_false_direction_and_red_are_real_detections(self):
        s=Telemetry(self.clock);s.set('lr_sign',False);s.set('signal',False);rows={m['id']:m for m in s.state()['missions']}
        self.assertEqual(rows['fork']['detected'],'왼쪽');self.assertEqual(rows['signal']['detected'],'빨강');self.now+=2.01;self.assertIsNone(s.get('signal'))
    def test_zero_speed_owner_is_still_controlling_and_done_unknown(self):
        s=Telemetry(self.clock);s.set('owner','crosswalk');s.set('cmd',dict(speed=0.,yaw_rate=0.));rows={m['id']:m for m in s.state()['missions']}
        self.assertTrue(rows['crosswalk']['control']);self.assertIsNone(rows['crosswalk']['active']);self.assertIsNone(rows['crosswalk']['done']);self.assertEqual(s.state()['command']['speed'],0.)
    def test_session_reset_clears_old_mapping(self):
        s=Telemetry(self.clock);s.set('parking_status',dict(mission_attempt=1,synchronization=dict(localization_restart_count=0)))
        s.markers.receive('/parking/slot_map',[marker()]);s.geometry['/parking/path']={}
        s.set('parking_status',dict(mission_attempt=2,synchronization=dict(localization_restart_count=0)))
        self.assertFalse(s.markers.records('/parking/slot_map'));self.assertNotIn('/parking/path',s.geometry)
    def test_leases_union_and_expiry(self):
        s=Leases(self.clock);s.touch('a','parking',['camera','yolo','lane']);s.touch('b','objavoid',['camera','mask'],'odom');groups,images=s.demand();self.assertEqual(groups,{'parking','objavoid'});self.assertEqual(images,{'camera','yolo','mask'});self.now+=3.1;self.assertEqual(s.demand(),(set(),set()))
    def test_events_are_bounded_and_nonfinite_values_sanitized(self):
        s=Telemetry(self.clock)
        for i in range(350):s.set('owner',str(i))
        self.assertEqual(len(s.events),300);s.set('white_ratio',float('nan'));self.assertIsNone(s.get('white_ratio'))

if __name__=='__main__':unittest.main()

import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from mission_core import Telemetry
from mission_runtime import select_domain,detect_domains,RuntimeMonitor

class RuntimeTests(unittest.TestCase):
    def test_explicit_domain_wins_and_single_robot_domain_autodetects(self):
        self.assertEqual(select_domain({'ROS_DOMAIN_ID':'97'},[{'domain':96}]),(97,'environment'))
        self.assertEqual(select_domain({},[{'domain':88}]),(88,'running_ros_processes'))
        self.assertEqual(select_domain({},[]),(99,'default'))
        self.assertEqual(select_domain({},[{'domain':88},{'domain':89}]),(99,'default'))
    def test_invalid_domain_fails_instead_of_silently_using_default(self):
        for value in ('abc','-1','233'):
            with self.assertRaises(ValueError):select_domain({'ROS_DOMAIN_ID':value},[])
    def test_same_user_proc_scan_excludes_web_and_does_not_return_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            def process(pid,args,env):
                d=root/str(pid);d.mkdir();(d/'cmdline').write_bytes(args);(d/'environ').write_bytes(env)
            process(999001,b'/usr/bin/python3\0/home/wego/2026-AutoRace/yolo_ws/src/yolo_detect.py\0',b'ROS_DOMAIN_ID=88\0SECRET=do-not-return\0')
            process(999002,b'/usr/bin/python3\0/home/wego/2026-AutoRace/web_viewer/mission_server.py\0',b'ROS_DOMAIN_ID=96\0')
            process(999003,b'/bin/bash\0-c\0echo /home/wego/2026-AutoRace/yolo_ws/test.py\0',b'ROS_DOMAIN_ID=11\0')
            process(999004,b'/opt/ros/humble/bin/ros2\0launch\0tensor_bringup\0mission_web.launch.py\0',b'ROS_DOMAIN_ID=0\0')
            self.assertEqual(detect_domains(root),[{'domain':88,'process_count':1,'processes':['yolo_detect.py']}])
    def test_domain_change_and_ip_change_emit_once_and_poll_is_throttled(self):
        now=[0.];observed=[96];urls=['http://192.168.0.106:8080'];state=Telemetry(clock=lambda:now[0]);scans=[]
        def scan():scans.append(now[0]);return [dict(domain=d,process_count=1,processes=['ros2']) for d in observed]
        runtime=RuntimeMonitor(state,96,8080,scanner=scan,address_reader=lambda *_:[dict(interface='wlan0',ip='192.168.0.106',url=url) for url in urls],clock=lambda:now[0])
        with contextlib.redirect_stdout(io.StringIO()) as log:
            runtime.poll();first=len(state.events);runtime.poll();self.assertEqual(len(scans),1)
            now[0]=5;observed[:]=[97];runtime.poll();self.assertTrue(runtime.snapshot()['domain_mismatch']);self.assertIn('96',runtime.snapshot()['warning']);self.assertIn('97',runtime.snapshot()['warning']);self.assertEqual(len(state.events),first+1)
            now[0]=10;runtime.poll();self.assertEqual(len(state.events),first+1)
            now[0]=15;urls[:]=['http://172.20.10.4:8080'];runtime.poll();self.assertEqual(len(state.events),first+2)
            self.assertIn('도메인 변경',log.getvalue());self.assertIn('172.20.10.4',log.getvalue())

if __name__=='__main__':unittest.main()

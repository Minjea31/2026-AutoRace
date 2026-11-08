import math
import unittest
from dataclasses import replace
from unittest.mock import patch
from autorace_objavoid.node import Config, OdomPose, RoadBand, AvoidanceMemory, Sector, Sectors, target_path


class TrackingTests(unittest.TestCase):
    def test_fixed_goal_and_countersteering(self):
        c = Config()
        pose = OdomPose(0., 0., 0., 1., 1., 'odom')
        memory = AvoidanceMemory()
        memory.select(1, pose, c, RoadBand(0., 1., 0., .78, 2))
        target = memory.target
        memory.select(1, replace(pose, y=.2), c)
        self.assertEqual(memory.target, target)
        self.assertLess(target.command(replace(pose, y=.4, yaw=.4), c), 0.)

    def test_bicycle_alignment_both_directions(self):
        c = Config()
        for side in (1, -1):
            pose = OdomPose(0., 0., 0., 1., 1., 'odom')
            memory = AvoidanceMemory()
            memory.select(side, pose, c, RoadBand(0., 1., 0., .78, 2))
            sector = Sector(3., 3)
            for _ in range(1000):
                road = RoadBand(-math.sin(pose.yaw), math.cos(pose.yaw), pose.y, .78, 2)
                scan = Sectors(sector, Sector(.7, 3), sector, 1., 1., road=road)
                result, _ = target_path(scan, c, memory, pose)
                self.assertGreater(result[1], 0.)
                speed, angle = result[1:3]
                yaw = pose.yaw + speed * math.tan(angle) / c.wheelbase * .05
                mid = (pose.yaw + yaw) / 2
                pose = replace(pose, x=pose.x + speed * math.cos(mid) * .05,
                               y=pose.y + speed * math.sin(mid) * .05, yaw=yaw)
                if memory.target.reached(pose, c):
                    break
            else:
                self.fail('target not reached')

    def test_blocked_target_stops_without_added_recovery(self):
        c = Config()
        pose = OdomPose(0., 0., 0., 1., 1., 'odom')
        memory = AvoidanceMemory()
        memory.select(1, pose, c)
        sector = Sector(3., 3)
        scan = Sectors(sector, Sector(.7, 3), sector, 1., 1.)
        with patch('autorace_objavoid.node.path_clear', return_value=False) as check:
            result, _ = target_path(scan, c, memory, pose)
        self.assertEqual(result[0], 'STOP_TARGET_BLOCKED')
        self.assertEqual(check.call_count, 4)

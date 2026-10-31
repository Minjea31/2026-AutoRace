"""Optional parking-only adapter for a mux workspace that is still unimplemented.

Never run together with the real mux: both would publish /cmd_vel. This is an
explicit development option and is not started by the simulator's default launch.
"""
import argparse
import math
import sys
import time

from .config import load_config
from .model import Simulation


def drive_to_twist(speed, steering, wheelbase, max_speed, max_steering):
    if not all(math.isfinite(x) for x in (speed, steering)):
        return 0., 0.
    v = max(-max_speed,min(max_speed,speed))
    delta = max(-max_steering,min(max_steering,steering))
    return v,v*math.tan(delta)/wheelbase


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.utilities import remove_ros_args
    from geometry_msgs.msg import Twist
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
    except ImportError:
        raise SystemExit('Optional adapter requires ackermann_msgs. Install ros-humble-ackermann-msgs '
                         'or use the existing mux publishing /cmd_vel.')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    options = parser.parse_args(remove_ros_args(['parking_adapter',*(sys.argv[1:] if args is None else args)])[1:])
    cfg = load_config(options.config)
    steering_limit = Simulation(cfg).max_steering

    class Adapter(Node):
        def __init__(self):
            super().__init__('parking_sim_adapter')
            if self.has_parameter('use_sim_time'):
                from rclpy.parameter import Parameter
                self.set_parameters([Parameter('use_sim_time',value=True)])
            else:
                self.declare_parameter('use_sim_time',True)
            self.value = (0.,0.)
            self.received = -math.inf
            self.source_stamp = None
            self.publisher = self.create_publisher(Twist,cfg['topics']['cmd_vel'],QoSProfile(depth=1))
            self.create_subscription(AckermannDriveStamped,cfg['topics']['parking_command'],self.receive,QoSProfile(depth=1))
            self.create_timer(1./30,self.tick,clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
            self.get_logger().info('Parking-only adapter active; do not run together with mux_ws')

        def receive(self,msg):
            now = self.get_clock().now().nanoseconds/1e9
            source = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if now-source > .25 or source-now > .05:
                return
            v = cfg['vehicle']
            self.value = drive_to_twist(msg.drive.speed,msg.drive.steering_angle,v['wheelbase'],v['max_speed'],steering_limit)
            self.received,self.source_stamp = time.monotonic(),source

        def tick(self):
            now = self.get_clock().now().nanoseconds/1e9
            fresh = (time.monotonic()-self.received <= .25 and self.source_stamp is not None
                     and -.05 <= now-self.source_stamp <= .25)
            v,w = self.value if fresh else (0.,0.)
            msg = Twist()
            msg.linear.x,msg.angular.z = v,w
            self.publisher.publish(msg)

    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

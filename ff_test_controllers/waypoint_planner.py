import math
import numpy as np

from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint, OffboardControlMode

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner')
        self.control_mode_pub = self.create_publisher(OffboardControlMode,
                                                      '/fmu/in/offboard_control_mode',
                                                      10)
        msg = OffboardControlMode()
        msg.position = True
        self.control_mode_pub.publish(msg)

        self.waypoint_publisher = self.create_publisher(TrajectorySetpoint,
                                                        '/fmu/in/trajectory_setpoint',
                                                        10)
        self.waypoint_publisher
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.t0 = self.get_clock().now().nanoseconds

        self.wp_times = np.array([0,10,20,30,40])
        self.wps = np.array([[0,0,0, 0],
                             [1,0,0, 0],
                             [1,1,0, 0],
                             [0,1,0, 0],
                             [0,0,0, 0]],dtype=float)
        
    def timer_callback(self):
        msg = TrajectorySetpoint()

        ti = self.get_clock().now().nanoseconds
        dt = (ti - self.t0) * 1e-9
        wp_idx = np.searchsorted(self.wp_times, dt, side='right') - 1

        # fill in the message
        msg.position = self.wps[wp_idx][0:3].tolist()
        msg.yaw      = self.wps[wp_idx][3]

        # publish it
        self.waypoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing waypoint: {self.wps[wp_idx]}")


def main():
    rclpy.init()
    node = WaypointPlanner()
    rclpy.spin(node)

    # Destroy node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
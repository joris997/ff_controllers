from px4_msgs.msg import VehicleOdometry

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_listener = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.state_estimator_callback,
            qos_profile
        )
        self.odom_listener
    
    def state_estimator_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.timestamp)
        self.position = np.array(msg.position)
        self.q = np.array(msg.q)


def main():
    rclpy.init()
    node = StateEstimator()
    rclpy.spin(node)

    # Destroy node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
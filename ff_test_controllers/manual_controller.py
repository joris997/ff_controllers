#!/usr/bin/env python3

import math
import numpy as np
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
    VehicleLocalPosition, VehicleStatus, VehicleThrustSetpoint, VehicleOdometry, \
    VehicleTorqueSetpoint, VehicleAttitude, VehicleLocalPosition

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
import tf_transformations as tf_trans

# def quaternion_to_euler(quaternion):
#     """
#     Convert a quaternion into euler angles (roll, pitch, yaw)
#     roll is rotation around x in radians (counterclockwise)
#     pitch is rotation around y in radians (counterclockwise)
#     yaw is rotation around z in radians (counterclockwise)
#     """
#     x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.atan2(t0, t1)
    
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.asin(t2)
    
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw_z = math.atan2(t3, t4)
    
#     return np.array([roll_x, pitch_y, yaw_z])


class ManualController(Node):
    def __init__(self):
        super().__init__('thruster')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,
                                                               '/fmu/in/offboard_control_mode',
                                                               qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
                                                         '/fmu/in/vehicle_command',
                                                         qos_profile)
        self.thrust_pub = self.create_publisher(VehicleThrustSetpoint,
                                                '/fmu/in/vehicle_thrust_setpoint',
                                                qos_profile)
        self.torque_pub = self.create_publisher(VehicleTorqueSetpoint,
                                                '/fmu/in/vehicle_torque_setpoint',
                                                qos_profile)
        
        # Create subscribers
        self.odometry_sub = self.create_subscription(VehicleOdometry,
                                                     '/fmu/out/vehicle_odometry',
                                                     self.vehicle_odometry_callback,
                                                     qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus,
                                                           '/fmu/out/vehicle_status',
                                                           self.vehicle_status_callback,
                                                           qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude,
                                                     '/fmu/out/vehicle_attitude',
                                                     self.vehicle_attitude_callback,
                                                     qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                           '/fmu/out/vehicle_local_position',
                                                           self.vehicle_local_position_callback,
                                                           qos_profile)

        # # Services
        # self.set_pose_srv = self.create_service(SetPose, '/set_pose', self.add_set_pos_callback)

        # Initialize variables
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])

        # keyboard controller
        self.thrust = [0.0, 0.0, 0.0]
        self.torque = [0.0, 0.0, 0.0]
        self.step = 0.1

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.key_mapping = {
            'q': (0, self.step), 'a': (0, -self.step),
            'w': (1, self.step), 's': (1, -self.step),
            'e': (2, self.step), 'd': (2, -self.step),
            'r': (0, self.step), 'f': (0, -self.step),
            't': (1, self.step), 'g': (1, -self.step),
            'y': (2, self.step), 'h': (2, -self.step)
        }
        self.thread = threading.Thread(target=self.keyboard_listener)
        self.thread.start()

        # main loop
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz



    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.thrust_and_torque = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)



    def timer_callback(self):
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # convert thrust and torque from body frame to world frame
            msg = VehicleThrustSetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.xyz = self.thrust
            self.thrust_pub.publish(msg)

            msg = VehicleTorqueSetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.xyz = self.torque
            self.torque_pub.publish(msg)

            print("Thrust: ", self.thrust)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def keyboard_listener(self):
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in self.key_mapping:
                        index, value = self.key_mapping[key]
                        if key in 'qawsed':  # for thrust
                            self.thrust[index] += value
                        elif key in 'rftgyh':  # for torque
                            self.torque[index] += value
                        self.get_logger().info(f'Thrust: {self.thrust}, Torque: {self.torque}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.thread.join()


def main(args=None) -> None:
    print("Starting manual control node...")
    rclpy.init(args=args)
    node = ManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
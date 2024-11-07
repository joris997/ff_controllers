#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
    VehicleLocalPosition, VehicleStatus, VehicleThrustSetpoint, VehicleOdometry, \
    VehicleTorqueSetpoint, VehicleAttitude, VehicleLocalPosition
from mpc_msgs.srv import SetPose

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
import tf_transformations as tf_trans

class PID_controller(Node):
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

        # Services
        self.set_pose_srv = self.create_service(SetPose, '/set_pose', self.add_set_pos_callback)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize variables
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_attitude = np.array([0.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])

        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        # Timer for commands being sent
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.desired_odometry = VehicleOdometry()
        self.desired_odometry.position = [0., 0., 0.]
        self.desired_odometry.q = [0., 0., 0., 0.]
        self.desired_odometry.velocity = [0., 0., 0.]
        self.desired_odometry.angular_velocity = [0., 0., 0.]

        
        
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry


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
            force_world, torque_world = self.compute_force_torque()
            force_body, torque_body = self.convert_to_body_frame(force_world,torque_world)
            # print type of force_body and torque_body
            print(f"force_body: {force_body}"
                  f"torque_body: {torque_body}")

            msg = VehicleThrustSetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.xyz[0] = force_body[0]
            msg.xyz[1] = force_body[1]
            msg.xyz[2] = force_body[2]
            self.thrust_pub.publish(msg)
            # self.get_logger().info(f"Publishing thrust: {msg.xyz}")

            msg = VehicleTorqueSetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.xyz[0] = torque_body[0]
            msg.xyz[1] = torque_body[1]
            msg.xyz[2] = torque_body[2]
            self.torque_pub.publish(msg)
            # self.get_logger().info(f"Publishing torque: {msg.xyz}")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def compute_force_torque(self):
        # Desired force and torque are computed in the robot frame!

        # actual
        x = np.array(self.vehicle_odometry.position)
        q = tf_trans.euler_from_quaternion(self.vehicle_odometry.q.tolist())
        dx = np.array(self.vehicle_odometry.velocity)
        dq = np.array(self.vehicle_odometry.angular_velocity)

        # desired
        x_des = np.array(self.desired_odometry.position)
        q_des = np.array(tf_trans.euler_from_quaternion(self.desired_odometry.q.tolist()))
        dx_des = np.array(self.desired_odometry.velocity)
        dq_des = np.array(self.desired_odometry.angular_velocity)

        # PD terms
        Px = .1
        Dx = 1

        Pq = .1
        Dq = 1

        force = -Px*(x - x_des) - Dx*(dx - dx_des)
        torque = -Pq*(q - q_des) - Dq*(dq - dq_des)
        print(f"x error: {x - x_des}")
        print(f"dx error: {dx - dx_des}")
        print("---")
        print(f"q error: {q - q_des}")
        print(f"dq error: {dq - dq_des}")

        return force, torque

    def convert_to_body_frame(self, force, torque):
        # Convert force and torque to body frame
        t = self.tf_buffer.lookup_transform('snap', 'map', rclpy.time.Time())

        translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        quaternion_matrix = tf_trans.quaternion_matrix([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        rotation_matrix = quaternion_matrix[:3,:3]

        t_force = np.dot(rotation_matrix, force)
        t_torque = np.dot(rotation_matrix, torque)

        return t_force, t_torque

    def add_set_pos_callback(self, request, response):
        self.desired_odometry.position = [request.pose.position.x, request.pose.position.y, request.pose.position.z]
        return response


def main(args=None) -> None:
    print("Starting manual control node...")
    rclpy.init(args=args)
    node = Thruster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
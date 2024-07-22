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


class Thruster(Node):
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

        # Services
        self.set_pose_srv = self.create_service(SetPose, '/set_pose', self.add_set_pos_callback)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize variables
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
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
            force_world, torque_world = self.compute_force_torque()
            force_body, torque_body = self.convert_to_body_frame(force_world,torque_world)
            # print type of force_body and torque_body
            print(f"force_body: {force_body}"
                  f"torque_body: {torque_body}")
            print(f"force_body type: {type(force_body)}")
            print(f"torque_body type: {type(torque_body)}")

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
        x = self.vehicle_local_position
        q = tf_trans.euler_from_quaternion(self.vehicle_attitude)
        dx = self.vehicle_local_velocity
        dq = np.array(self.vehicle_odometry.angular_velocity)

        # desired
        x_des = np.array(self.desired_odometry.position)
        q_des = np.array(tf_trans.euler_from_quaternion(self.desired_odometry.q.tolist()))
        dx_des = np.array(self.desired_odometry.velocity)
        dq_des = np.array(self.desired_odometry.angular_velocity)

        # PD terms
        Px = 0.001
        Dx = 0.01

        Pq = 0.001
        Dq = 0.01

        force = -Px*(x - x_des) - Dx*(dx - dx_des)
        torque = -Pq*(q - q_des) - Dq*(dq - dq_des)
        print(f"error: {x - x_des}")
        print(f"q:     {q}")

        # force[2] = 0.0
        force[0], force[1], force[2] = force[0], -force[1], -force[2]
        # torque[2] = torque[0]
        # torque[0:2] = np.zeros((2,))

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
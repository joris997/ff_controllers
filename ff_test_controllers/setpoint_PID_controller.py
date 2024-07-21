#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
    VehicleLocalPosition, VehicleStatus, VehicleThrustSetpoint, VehicleOdometry, \
    VehicleTorqueSetpoint

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return np.array([roll_x, pitch_y, yaw_z])



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
        
        # Initialize variables
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        # Timer for commands being sent
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.desired_odometry = VehicleOdometry()
        self.desired_odometry.position = [0., 0., 0.0]
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
            force, torque = self.compute_force_torque()
            print(f"force: {force}")
            # fill in the message
            msg = VehicleThrustSetpoint()
            msg.xyz = force.tolist()
            self.thrust_pub.publish(msg)
            # self.get_logger().info(f"Publishing thrust: {msg.xyz}")

            msg = VehicleTorqueSetpoint()
            msg.xyz = torque.tolist()
            self.torque_pub.publish(msg)
            # self.get_logger().info(f"Publishing torque: {msg.xyz}")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def compute_force_torque(self):
        # actual
        x = np.array(self.vehicle_odometry.position)
        q = quaternion_to_euler(self.vehicle_odometry.q)
        dx = np.array(self.vehicle_odometry.velocity)
        dq = np.array(self.vehicle_odometry.angular_velocity)

        # desired
        x_des = np.array(self.desired_odometry.position)
        q_des = quaternion_to_euler(self.desired_odometry.q)
        dx_des = np.array(self.desired_odometry.velocity)
        dq_des = np.array(self.desired_odometry.angular_velocity)

        # PD terms
        Px = 0.1
        Dx = 10

        Pq = 1
        Dq = 0.1

        force = -Px*(x - x_des) - Dx*(dx - dx_des)
        torque = -Pq*(q - q_des) - Dq*(dq - dq_des)
        print(f"error: {x - x_des}")

        force[2] = 0.0
        force[0], force[1] = force[1], -force[0]
        torque[0:2] = np.zeros((2,))

        return force, np.array([0.0,0.0,0.0])# torque.flatten()

def main(args=None) -> None:
    print("Starting manual control node...")
    rclpy.init(args=args)
    node = Thruster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
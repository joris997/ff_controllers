#!/usr/bin/env python

__author__ = "Joris Verhagen"
__contact__ = "jorisv@kth.se"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleThrustSetpoint

from ff_my_msgs.srv import SetPose, SetOdometry

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.controller_type = self.declare_parameter('controller', 'PID').value

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        self.angular_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.vehicle_angular_velocity_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        # Services
        self.set_pose_srv = self.create_service(SetPose, '/set_pose', self.set_pose_srv)
        self.set_state_srv = self.create_service(SetState, '/set_state', self.set_odometry_srv)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_rates_setpoint = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.publisher_direct_actuator = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.publisher_thrust_setpoint = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.publisher_torque_setpoint = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile)
        self.reference_pub = self.create_publisher(Marker, "/px4_mpc/reference", 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vehicle_status = VehicleStatus()

        # Create different controller objects
        if self.controller_type == 'PID':
            from ff_controllers.PID_controller import PID_controller
            self.controller = PID_controller()
        elif self.controller_type == 'HJI':
            from ff_controllers.HJI_controller import HJI_controller
            self.controller = HJI_controller()

        # actual state
        self.x = np.zeros((13,1))
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])

        # desired state
        self.x_des = np.zeros((13,1))
        self.setpoint_position = np.array([1.0, 0.0, 0.0])
        self.setpoint_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.setpoint_angular_velocity = np.array([0.0, 0.0, 0.0])

    
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

    def vehicle_angular_velocity_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_angular_velocity_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def set_pose_srv(self, request, response):
        # Request: Pose msg (geometry_msgs/Pose)
        # Response: bool 
        self.setpoint_position[0] = request.pose.position.x
        self.setpoint_position[1] = request.pose.position.y
        self.setpoint_position[2] = request.pose.position.z
        self.setpoint_velocity[0], self.setpoint_velocity[1], self.setpoint_velocity[2] = 0.0, 0.0, 0.0
        self.setpoint_attitude[0], self.setpoint_attitude[1], self.setpoint_attitude[2], self.setpoint_attitude[3] = request.pose.orientation.w, request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z
        self.setpoint_angular_velocity[0], self.setpoint_angular_velocity[1], self.setpoint_angular_velocity[2] = 0.0, 0.0, 0.0

        self.x_des = np.array([self.setpoint_position[0], self.setpoint_position[1], self.setpoint_position[2],
                               self.setpoint_velocity[0], self.setpoint_velocity[1], self.setpoint_velocity[2],
                               self.setpoint_attitude[0], self.setpoint_attitude[1], self.setpoint_attitude[2], self.setpoint_attitude[3],
                               self.setpoint_angular_velocity[0], self.setpoint_angular_velocity[1], self.setpoint_angular_velocity[2]]).reshape(13, 1)

        # now we need to check if the controller has computed what needs to be computed
        # for PID, this is trivial but for HJI, we need to compute the reachable set from x to x_des
        self.controller.prepare_controller(self.x, self.x_des)
        return response

    def set_odometry_srv(self, request, response):
        # Request: Odometry msg (nav_msgs/Odometry)
        # Response: bool
        self.setpoint_position[0] = request.pose.position.x
        self.setpoint_position[1] = request.pose.position.y
        self.setpoint_position[2] = request.pose.position.z
        self.setpoint_velocity[0] = request.twist.linear.x
        self.setpoint_velocity[1] = request.twist.linear.y
        self.setpoint_velocity[2] = request.twist.linear.z
        self.setpoint_attitude[0] = request.pose.orientation.w
        self.setpoint_attitude[1] = request.pose.orientation.x
        self.setpoint_attitude[2] = request.pose.orientation.y
        self.setpoint_attitude[3] = request.pose.orientation.z
        self.setpoint_angular_velocity[0] = request.twist.angular.x
        self.setpoint_angular_velocity[1] = request.twist.angular.y
        self.setpoint_angular_velocity[2] = request.twist.angular.z

        self.x_des = np.array([self.setpoint_position[0], self.setpoint_position[1], self.setpoint_position[2],
                               self.setpoint_velocity[0], self.setpoint_velocity[1], self.setpoint_velocity[2],
                               self.setpoint_attitude[0], self.setpoint_attitude[1], self.setpoint_attitude[2], self.setpoint_attitude[3],
                               self.setpoint_angular_velocity[0], self.setpoint_angular_velocity[1], self.setpoint_angular_velocity[2]]).reshape

        # now we need to check if the controller has computed what needs to be computed
        # for PID, this is trivial but for HJI, we need to compute the reachable set from x to x_des
        self.controller.prepare_controller(self.x, self.x_des)
        return response


    def publish_reference(self, pub, reference):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        msg.ns = "arrow"
        msg.id = 1
        msg.type = Marker.SPHERE
        msg.scale.x = 0.5
        msg.scale.y = 0.5
        msg.scale.z = 0.5
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        msg.pose.position.x = reference[0]
        msg.pose.position.y = reference[1]
        msg.pose.position.z = reference[2]
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        pub.publish(msg)

    def publish_wrench_setpoint(self, u_pred):
        thrust_outputs_msg = VehicleThrustSetpoint()
        thrust_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        torque_outputs_msg = VehicleTorqueSetpoint()
        torque_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        thrust_outputs_msg.xyz = [u_pred[0], -u_pred[1], -0.0]
        torque_outputs_msg.xyz = [0.0, -0.0, -u_pred[2]]

        self.publisher_thrust_setpoint.publish(thrust_outputs_msg)
        self.publisher_torque_setpoint.publish(torque_outputs_msg)

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = False
        offboard_msg.thrust_and_torque = True
        self.publisher_offboard_mode.publish(offboard_msg)

        error_position = self.vehicle_local_position - self.setpoint_position

        self.x = np.array([self.vehicle_local_position[0],
                            self.vehicle_local_position[1],
                            self.vehicle_local_position[2],
                            self.vehicle_local_velocity[0],
                            self.vehicle_local_velocity[1],
                            self.vehicle_local_velocity[2],
                            self.vehicle_attitude[0],
                            self.vehicle_attitude[1],
                            self.vehicle_attitude[2],
                            self.vehicle_attitude[3],
                            self.vehicle_angular_velocity[0],
                            self.vehicle_angular_velocity[1],
                            self.vehicle_angular_velocity[2]]).reshape(13, 1)

        u = self.controller.solve(x, self.x_des)

        self.publish_reference(self.reference_pub, self.setpoint_position)
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_wrench_setpoint(u_pred)
    

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
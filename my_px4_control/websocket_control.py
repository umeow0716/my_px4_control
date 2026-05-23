#!/usr/bin/env python3

import json
import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String

from px4_msgs.msg import \
    OffboardControlMode, \
    TrajectorySetpoint, \
    VehicleCommand, \
    VehicleLocalPosition, \
    VehicleStatus, \
    VehicleAttitudeSetpoint


class VehicleState(str, Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    HOVER = "hover"
    LANDING = "landing"
    LANDED = "landed"


class WebSocketControl(Node):
    """PX4 offboard control node using vx/wz commands from websocket_publisher."""

    def __init__(self) -> None:
        super().__init__('websocket_control_takeoff_and_track')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.websocket_cmd_subscriber = self.create_subscription(
            String, '/my_px4_control/websocket_cmd', self.websocket_cmd_callback, 10)

        self.state = VehicleState.TAKEOFF

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -2.2

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.last_websocket_cmd_time_sec = 0.0
        self.websocket_cmd_timeout_sec = 0.3

        self.trajectory_setpoint_msg = TrajectorySetpoint()
        self.trajectory_setpoint_msg.position[2] = self.takeoff_height
        self.trajectory_setpoint_msg.velocity[0] = self.vx
        self.trajectory_setpoint_msg.velocity[1] = self.vy
        self.trajectory_setpoint_msg.velocity[2] = self.vz
        self.trajectory_setpoint_msg.yaw = math.nan
        self.trajectory_setpoint_msg.yawspeed = self.wz

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def websocket_cmd_callback(self, msg: String):
        if self.state != VehicleState.HOVER:
            return

        try:
            data = json.loads(msg.data)
            self.vx = float(data.get('vx', 0.0))
            self.wz = float(data.get('wz', 0.0))
            self.last_websocket_cmd_time_sec = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            return

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = self.state == VehicleState.HOVER
        msg.acceleration = False
        msg.attitude = self.state == VehicleState.TAKEOFF
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if now_sec - self.last_websocket_cmd_time_sec > self.websocket_cmd_timeout_sec:
            self.vx = 0.0
            self.wz = 0.0

        self.trajectory_setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_msg.position[0] = np.nan
        self.trajectory_setpoint_msg.position[1] = np.nan
        self.trajectory_setpoint_msg.position[2] = self.takeoff_height
        self.trajectory_setpoint_msg.velocity[0] = self.vx
        self.trajectory_setpoint_msg.velocity[1] = self.vy
        self.trajectory_setpoint_msg.velocity[2] = self.vz
        self.trajectory_setpoint_msg.yaw = math.nan
        self.trajectory_setpoint_msg.yawspeed = self.wz
        self.trajectory_setpoint_publisher.publish(self.trajectory_setpoint_msg)

    def publish_attitude_setpoint(self, z: float):
        msg = VehicleAttitudeSetpoint()
        msg.thrust_body[2] = z
        self.attitude_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
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
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.state == VehicleState.TAKEOFF:
            self.publish_position_setpoint()

        if self.state == VehicleState.TAKEOFF and self.vehicle_local_position.z <= -0.5:
            self.state = VehicleState.HOVER
            self.vx = 0.0
            self.wz = 0.0
            self.last_websocket_cmd_time_sec = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info('Takeoff complete. Start accepting websocket control commands.')

        if self.state == VehicleState.HOVER:
            self.publish_position_setpoint()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting websocket control node...')
    rclpy.init(args=args)
    websocket_control = WebSocketControl()
    rclpy.spin(websocket_control)
    websocket_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

#!/usr/bin/env python3

import math
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition


class VehiclePositionLogger(Node):
    """Only subscribe to VehicleLocalPosition and save 4 plots to PNG."""

    def __init__(self) -> None:
        super().__init__('vehicle_position_logger')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )

        self.start_time = None
        self.t_list = []
        self.x_list = []
        self.y_list = []
        self.z_list = []

        self.msg_count = 0

        self.get_logger().info('Started listening to /fmu/out/vehicle_local_position')

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now_sec

        t = now_sec - self.start_time

        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)

        if any(math.isnan(v) for v in [x, y, z]):
            return

        self.t_list.append(t)
        self.x_list.append(x)
        self.y_list.append(y)
        self.z_list.append(z)

        self.msg_count += 1

        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f"Samples={self.msg_count}, x={x:.3f}, y={y:.3f}, z={z:.3f}, t={t:.2f}s"
            )

    def save_all_plots(self) -> None:
        if len(self.t_list) == 0:
            print('[WARN] No valid VehicleLocalPosition data received, no PNG generated.')
            return

        # 1. x-y
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(self.x_list, self.y_list, linewidth=1.5)
        ax.scatter(self.x_list[0], self.y_list[0], s=40, label='start')
        ax.scatter(self.x_list[-1], self.y_list[-1], s=40, label='end')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('XY Trajectory')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig('vehicle_xy.png', dpi=200)
        plt.close(fig)
        print(f'[INFO] Saved: {Path("vehicle_xy.png").resolve()}')

        # 2. z-t
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(self.t_list, self.z_list, linewidth=1.5)
        ax.set_xlabel('t (s)')
        ax.set_ylabel('z (m)')
        ax.set_title('Z vs Time')
        ax.grid(True)
        fig.tight_layout()
        fig.savefig('vehicle_zt.png', dpi=200)
        plt.close(fig)
        print(f'[INFO] Saved: {Path("vehicle_zt.png").resolve()}')

        # 3. x-t
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(self.t_list, self.x_list, linewidth=1.5)
        ax.set_xlabel('t (s)')
        ax.set_ylabel('x (m)')
        ax.set_title('X vs Time')
        ax.grid(True)
        fig.tight_layout()
        fig.savefig('vehicle_xt.png', dpi=200)
        plt.close(fig)
        print(f'[INFO] Saved: {Path("vehicle_xt.png").resolve()}')

        # 4. y-t
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(self.t_list, self.y_list, linewidth=1.5)
        ax.set_xlabel('t (s)')
        ax.set_ylabel('y (m)')
        ax.set_title('Y vs Time')
        ax.grid(True)
        fig.tight_layout()
        fig.savefig('vehicle_yt.png', dpi=200)
        plt.close(fig)
        print(f'[INFO] Saved: {Path("vehicle_yt.png").resolve()}')


def main(args=None) -> None:
    print('Starting vehicle position logger...')
    rclpy.init(args=args)
    node = VehiclePositionLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[INFO] KeyboardInterrupt received, saving plots...')
    finally:
        node.save_all_plots()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
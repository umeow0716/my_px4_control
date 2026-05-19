#!/usr/bin/env python3

import asyncio
import json
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from websockets.asyncio.client import connect
except ImportError as exc:
    raise RuntimeError(
        "Missing Python package: websockets. Install it with `pip install websockets` "
        "or `sudo apt install python3-websockets`."
    ) from exc


class WebSocketPublisher(Node):
    def __init__(self):
        super().__init__('websocket_publisher')

        self.websocket_url = 'ws://127.0.0.1:8890'
        self.reconnect_delay_sec = 1.0
        self.publish_topic = '/my_px4_control/websocket_cmd'
        self.heartbeat_timeout_sec = 0.5
        self.watchdog_period_sec = 0.05

        self.publisher_ = self.create_publisher(String, self.publish_topic, 10)
        self.stop_event = threading.Event()

        self.lock = threading.Lock()
        self.last_heartbeat_time = time.monotonic()
        self.safety_stop_active = False

        self.watchdog_timer = self.create_timer(self.watchdog_period_sec, self.watchdog_callback)

        self.worker_thread = threading.Thread(target=self._run_async_worker, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(
            f'WebSocket publisher started: {self.websocket_url} -> {self.publish_topic}, '
            f'heartbeat timeout={self.heartbeat_timeout_sec:.3f}s'
        )

    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()

    def _run_async_worker(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                self.get_logger().info(f'Connecting to {self.websocket_url}')

                async with connect(self.websocket_url) as websocket:
                    self.get_logger().info('WebSocket connected')
                    self.mark_heartbeat_received()

                    async for raw_msg in websocket:
                        if self.stop_event.is_set():
                            break
                        self.handle_websocket_message(raw_msg)

            except Exception as exc:
                self.get_logger().warn(f'WebSocket disconnected/error: {exc}')
                self.publish_zero_control('websocket disconnected')

            await asyncio.sleep(self.reconnect_delay_sec)

    def mark_heartbeat_received(self):
        with self.lock:
            self.last_heartbeat_time = time.monotonic()
            self.safety_stop_active = False

    def watchdog_callback(self):
        with self.lock:
            elapsed = time.monotonic() - self.last_heartbeat_time
            timeout = elapsed > self.heartbeat_timeout_sec
            was_safety_stop_active = self.safety_stop_active
            if timeout:
                self.safety_stop_active = True

        if not timeout:
            return

        if not was_safety_stop_active:
            self.get_logger().warn(
                f'No WebSocket heartbeat/control for {elapsed:.3f}s, force vx=0.0 wz=0.0'
            )

        self.publish_zero_control()

    def publish_zero_control(self, reason: str = ''):
        out_msg = String()
        out_msg.data = json.dumps({
            'vx': 0.0,
            'wz': 0.0,
        })
        self.publisher_.publish(out_msg)

        if reason:
            self.get_logger().warn(f'Force zero control: {reason}')

    def handle_websocket_message(self, raw_msg: Any):
        if isinstance(raw_msg, bytes):
            try:
                raw_msg = raw_msg.decode('utf-8')
            except UnicodeDecodeError:
                return

        if not isinstance(raw_msg, str):
            return

        try:
            data = json.loads(raw_msg)
        except json.JSONDecodeError:
            return

        msg_type = data.get('type')

        # 影像端目前每 0.05 秒送 flight_control；舊版若只送 heartbeat，也視為連線仍存活。
        if msg_type == 'heartbeat':
            self.mark_heartbeat_received()
            return

        if msg_type != 'flight_control':
            return

        self.mark_heartbeat_received()

        try:
            vx = float(data.get('vx', 0.0))
            wz = float(data.get('wz', 0.0))
        except (TypeError, ValueError):
            return

        out_msg = String()
        out_msg.data = json.dumps({
            'vx': vx,
            'wz': wz,
        })

        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

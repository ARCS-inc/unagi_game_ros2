#!/usr/bin/env python3

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger


class ViewerNode(Node):
    """Optional OpenCV window that consumes /game/image and emits ROS controls."""

    def __init__(self) -> None:
        super().__init__("unagi_game_viewer")
        self.declare_parameter("steer_hold_sec", 0.16)
        self.hold_sec = float(self.get_parameter("steer_hold_sec").value)
        self.sub = self.create_subscription(Image, "/game/image", self.on_image, 2)
        self.flap_pub = self.create_publisher(Vector3, "/game/flap_direction", 10)
        self.reset_client = self.create_client(Trigger, "/game/reset")
        self.pause_client = self.create_client(SetBool, "/game/pause")
        self.paused = False
        self.last_steer_time = 0.0
        self.last_y = 0.0
        self.latest_frame = None
        cv2.namedWindow("Unagi Game", cv2.WINDOW_NORMAL)
        self.timer = self.create_timer(0.02, self.ui_tick)
        self.publish_flap(1.0, 0.0)
        self.get_logger().info("Window controls: A/D steer, W straight, R reset, P pause, Q quit")

    def publish_flap(self, x: float, y: float) -> None:
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.flap_pub.publish(msg)

    def on_image(self, msg: Image) -> None:
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
            return
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.latest_frame = frame

    def ui_tick(self) -> None:
        if self.latest_frame is not None:
            cv2.imshow("Unagi Game", self.latest_frame)
        key = cv2.waitKey(1) & 0xFF
        now = time.monotonic()
        if key in (ord("a"), ord("A")):
            self.last_y = 0.75
            self.last_steer_time = now
            self.publish_flap(1.0, self.last_y)
        elif key in (ord("d"), ord("D")):
            self.last_y = -0.75
            self.last_steer_time = now
            self.publish_flap(1.0, self.last_y)
        elif key in (ord("w"), ord("W")):
            self.last_y = 0.0
            self.last_steer_time = now
            self.publish_flap(1.0, 0.0)
        elif key in (ord("r"), ord("R")):
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
        elif key in (ord("p"), ord("P")):
            self.paused = not self.paused
            if self.pause_client.service_is_ready():
                req = SetBool.Request()
                req.data = self.paused
                self.pause_client.call_async(req)
        elif key in (ord("q"), ord("Q")):
            rclpy.shutdown()
            return

        if self.last_y != 0.0 and now - self.last_steer_time > self.hold_sec:
            self.last_y = 0.0
            self.publish_flap(1.0, 0.0)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

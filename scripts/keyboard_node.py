#!/usr/bin/env python3

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class KeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("unagi_game_keyboard")
        self.declare_parameter("steer_hold_sec", 0.16)
        self.hold_sec = float(self.get_parameter("steer_hold_sec").value)
        self.pub = self.create_publisher(Vector3, "/game/flap_direction", 10)
        self.reset_client = self.create_client(Trigger, "/game/reset")
        self.pause_client = self.create_client(SetBool, "/game/pause")
        self.paused = False
        self.last_steer_time = 0.0
        self.last_y = 0.0
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd) if os.isatty(self.fd) else None
        if self.old_settings is not None:
            tty.setcbreak(self.fd)
        self.timer = self.create_timer(0.02, self.poll)
        self.get_logger().info("Keyboard: A/D steer, W straight, R reset, P pause, Q quit")
        self.publish_flap(1.0, 0.0)

    def destroy_node(self):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

    def publish_flap(self, x: float, y: float) -> None:
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.pub.publish(msg)

    def poll(self) -> None:
        now = time.monotonic()
        if self.old_settings is not None:
            while select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == "a":
                    self.last_y = 0.75
                    self.last_steer_time = now
                    self.publish_flap(1.0, self.last_y)
                elif ch == "d":
                    self.last_y = -0.75
                    self.last_steer_time = now
                    self.publish_flap(1.0, self.last_y)
                elif ch == "w":
                    self.last_y = 0.0
                    self.last_steer_time = now
                    self.publish_flap(1.0, 0.0)
                elif ch == "r":
                    if self.reset_client.service_is_ready():
                        self.reset_client.call_async(Trigger.Request())
                elif ch == "p":
                    self.paused = not self.paused
                    if self.pause_client.service_is_ready():
                        req = SetBool.Request()
                        req.data = self.paused
                        self.pause_client.call_async(req)
                elif ch == "q":
                    self.get_logger().info("Quit requested")
                    rclpy.shutdown()
                    return

        if self.last_y != 0.0 and now - self.last_steer_time > self.hold_sec:
            self.last_y = 0.0
            self.publish_flap(1.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == "__main__":
    main()

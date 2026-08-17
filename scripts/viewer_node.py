#!/usr/bin/env python3

from __future__ import annotations

import threading
import time
import tkinter as tk

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from PIL import Image as PILImage, ImageTk
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool, Trigger


class ViewerNode(Node):
    """Tkinter UI for consuming /game/image and emitting ROS controls."""

    def __init__(self) -> None:
        super().__init__("unagi_game_viewer")
        self.declare_parameter("steer_hold_sec", 0.16)
        self.hold_sec = float(self.get_parameter("steer_hold_sec").value)
        
        self.sub = self.create_subscription(RosImage, "/game/image", self.on_image, 2)
        self.flap_pub = self.create_publisher(Vector3, "/game/flap_direction", 10)
        self.reset_client = self.create_client(Trigger, "/game/reset")
        self.pause_client = self.create_client(SetBool, "/game/pause")
        self.paused = False
        
        self.last_steer_time = 0.0
        self.last_y = 0.0
        
        self.latest_pil_image = None
        self.image_lock = threading.Lock()
        
        self.timer = self.create_timer(0.02, self.check_steer_timeout)
        
        self.publish_flap(1.0, 0.0)
        self.get_logger().info("Window controls: A/D steer, W straight, R reset, P pause, Q quit")

    def publish_flap(self, x: float, y: float) -> None:
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.flap_pub.publish(msg)

    def on_image(self, msg: RosImage) -> None:
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
            return
            
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        
        if msg.encoding == "bgr8":
            frame = frame[..., ::-1]
            
        image = PILImage.fromarray(frame)
        
        with self.image_lock:
            self.latest_pil_image = image

    def handle_key(self, key_char: str) -> None:
        now = time.monotonic()
        key_char = key_char.lower()

        if key_char == "a":
            self.last_y = 0.75
            self.last_steer_time = now
            self.publish_flap(1.0, self.last_y)
        elif key_char == "d":
            self.last_y = -0.75
            self.last_steer_time = now
            self.publish_flap(1.0, self.last_y)
        elif key_char == "w":
            self.last_y = 0.0
            self.last_steer_time = now
            self.publish_flap(1.0, 0.0)
        elif key_char == "r":
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
        elif key_char == "p":
            self.paused = not self.paused
            if self.pause_client.service_is_ready():
                req = SetBool.Request()
                req.data = self.paused
                self.pause_client.call_async(req)

    def check_steer_timeout(self) -> None:
        now = time.monotonic()
        if self.last_y != 0.0 and now - self.last_steer_time > self.hold_sec:
            self.last_y = 0.0
            self.publish_flap(1.0, 0.0)


class App:
    """Tkinter Application Wrapper"""
    def __init__(self, root: tk.Tk, node: ViewerNode) -> None:
        self.root = root
        self.node = node
        self.root.title("Unagi Game")
        self.label = tk.Label(self.root)
        self.label.pack()
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.update_image()

    def on_key_press(self, event: tk.Event) -> None:
        char = event.char.lower()
        if char == "q":
            self.on_close()
        elif char in ("a", "d", "w", "r", "p"):
            self.node.handle_key(char)

    def update_image(self) -> None:
        with self.node.image_lock:
            img = self.node.latest_pil_image
            
        if img is not None:
            self.photo = ImageTk.PhotoImage(img)
            self.label.configure(image=self.photo)
            
        self.root.after(33, self.update_image)

    def on_close(self) -> None:
        self.root.quit()
        self.root.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ViewerNode()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    root = tk.Tk()
    app = App(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
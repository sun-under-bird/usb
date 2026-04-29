#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np


class StereoSplitCompressed(Node):

    def __init__(self):
        super().__init__('stereo_split_compressed')

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera1/image_raw/compressed',
            self.image_callback,
            10)

        self.left_pub = self.create_publisher(CompressedImage, '/camera/left/image_raw/compressed', 10)
        self.right_pub = self.create_publisher(CompressedImage, '/camera/right/image_raw/compressed', 10)

    def image_callback(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        half = len(data) // 2

        left_msg = CompressedImage()
        right_msg = CompressedImage()

        left_msg.header = msg.header
        left_msg.header.frame_id = 'camera_left'
        left_msg.format = msg.format
        left_msg.data = data[:half].tobytes()

        right_msg.header = msg.header
        right_msg.header.frame_id = 'camera_right'
        right_msg.format = msg.format
        right_msg.data = data[half:].tobytes()

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitCompressed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
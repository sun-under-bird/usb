#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class StereoSplit(Node):

    def __init__(self):
        super().__init__('stereo_split')

        self.bridge = CvBridge()
        self.use_compressed = False

        if self.use_compressed:
            self.sub = self.create_subscription(
                CompressedImage,
                '/camera1/image_raw/compressed',
                self.compressed_callback,
                10)
            self.left_pub = self.create_publisher(CompressedImage, '/camera/left/image_raw/compressed', 10)
            self.right_pub = self.create_publisher(CompressedImage, '/camera/right/image_raw/compressed', 10)
        else:
            self.sub = self.create_subscription(
                Image,
                '/camera1/image_raw',
                self.image_callback,
                10)
            self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
            self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)

    def compressed_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        height, width = cv_image.shape[:2]
        half_width = width // 2

        left_img = cv_image[:, :half_width]
        right_img = cv_image[:, half_width:]

        left_msg = CompressedImage()
        right_msg = CompressedImage()

        left_msg.header = msg.header
        left_msg.header.frame_id = 'camera_left'
        left_msg.format = 'jpeg'
        left_msg.data = np.array(cv2.imencode('.jpg', left_img)[1]).tobytes()

        right_msg.header = msg.header
        right_msg.header.frame_id = 'camera_right'
        right_msg.format = 'jpeg'
        right_msg.data = np.array(cv2.imencode('.jpg', right_img)[1]).tobytes()

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        height, width = cv_image.shape[:2]
        half_width = width // 2

        left_img = cv_image[:, :half_width]
        right_img = cv_image[:, half_width:]

        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding=msg.encoding)
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding=msg.encoding)

        left_msg.header = msg.header
        left_msg.header.frame_id = 'camera_left'
        right_msg.header = msg.header
        right_msg.header.frame_id = 'camera_right'

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoSplit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


import numpy as np

#Global variables
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_key = (1 << 6) | (1 << 2) | 1
isAbove = False


def listener_callback(image_data):
    global isAbove

    # Convert ROS Image message to OpenCV image
    cv_image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
    
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, ARUCO_DICT)
    horizon = cv_image.shape[0] // 2

    if marker_corners is not None and marker_ids is not None:
        for corner, id in zip(marker_corners, marker_ids):
            if id == aruco_key:
                x, y = np.mean(corner[0], axis=0) # Calculates center points coords based on corner coordinates
                isAbove = (y < horizon) 

    node = Node('camera_node')
    text = "Above" if isAbove else "Below"
    node.get_logger().info(text)

    # Display image
    cv2.imshow("camera", cv_image)

    # Stop to show the image
    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    # Create the node
    node = Node('camera_node')
    # Log information into the console
    node.get_logger().info('Hello node')
    # Create the subscriber. This subscriber will receive an Image
    # from the image_raw topic. The queue size is 10 messages.
    subscription = node.create_subscription(Image,'image_raw',listener_callback,10)
    
    # Spin the node so the callback function is called.
    rclpy.spin(node)
    # Spin the node so the callback function is called.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
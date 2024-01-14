#!/usr/bin/env python3

# node
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

# Msgs
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header


# Cv2
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageAnalyzerNode(Node):
    def __init__(self):
        super().__init__('image_analyzer_node')
        self.init_image_analyzer();
        self.init_controller()
        self.get_logger().info("Image analyzer node initalised.")

    def init_image_analyzer(self):
        self.subscription = self.create_subscription(Image, 'image_raw', self.image_raw_callback, 10)
        self.bridge = CvBridge()
        self.isAbove = False
        self.markerFound = False
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_key = (1 << 6) | (1 << 2) | 1

    def init_controller(self):
        # Define the joint names for UR3
        self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.travel_time = 2.00
        self.timer = self.create_timer(self.travel_time, self.publish_trajectory)

    # On timer callback, sends trajectory to UR3 controller
    def publish_trajectory(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header = Header()
        joint_trajectory.joint_names = [
                                        "shoulder_pan_joint",
                                        "shoulder_lift_joint",
                                        "elbow_joint",
                                        "wrist_1_joint",
                                        "wrist_2_joint",
                                        "wrist_3_joint"
                                    ]

        point = JointTrajectoryPoint()

        if self.isAbove:
            point.positions = [-1.57, -1.73, -2.2, -0.81, 1.59, -0.031] 
        else:
            point.positions = [0.0, -1.73, -2.2, -0.81, 1.59, -0.031] 

        point.velocities = []
        point.effort = []
        point.time_from_start = Duration(sec=int(self.travel_time), nanosec=0)
        
        joint_trajectory.points.append(point)

        self.publisher.publish(joint_trajectory)

    # On receiving image on topic, analize image, set flags for controller, logs only on change
    def image_raw_callback(self, image_data):
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        horizon = cv_image.shape[0] // 2

        # Detect aruco markers
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict)
        if marker_corners is not None and marker_ids is not None: # any marker was found
        
            if not(self.markerFound):
                self.get_logger().info("Marker has been found.")
                self.markerFound = True

            for corner, id in zip(marker_corners, marker_ids):
                if id == self.aruco_key:
                    # Calculates center points coords based on corner coordinates of marker
                    x, y = np.mean(corner[0], axis=0) 

                    if (y < horizon) and (self.isAbove == False): # Marker above horizon (middle)
                        self.get_logger().info("Marker is above horizon.")
                        self.isAbove = True
                    elif (y >= horizon) and (self.isAbove == True): # Marker below horizon
                        self.get_logger().info("Marker is below horizon.")
                        self.isAbove = False
        else:
            if self.markerFound:
                self.get_logger().info("Lost marker.")
                self.markerFound = False
                
        # Show camera image in window
        cv2.imshow("img", cv_image)
        cv2.waitKey(1)

def main(args=None):
    # Init node
    rclpy.init(args=args)
    node = ImageAnalyzerNode()
    rclpy.spin(node)

    # Destroy node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
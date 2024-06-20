#!/usr/bin/env python3
"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /aruco_image (sensor_msgs.msg.Image)
       Annotated image with marker locations and ids, with markers drawn on it

Parameters:
    marker_size - size of the markers in meters (default .065)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /camera/camera_info)
    camera_frame - camera optical frame to use (default "camera_depth_optical_frame")
    detected_markers_topic - topic to publish detected markers (default /aruco_markers)
    markers_visualization_topic - topic to publish markers visualization (default /aruco_poses)
    output_image_topic - topic to publish annotated image (default /aruco_image)

Author: Simone Giampà
Version: 2024-01-29

"""

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters

# Python imports
import numpy as np
import cv2

import os
# Local imports for custom defined functions
#from aruco_pose_estimation.utils import ARUCO_DICT
#from aruco_pose_estimation.pose_estimation import pose_estimation

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")
        self.img_idx = 0
        self.initialize_parameters()

        if not os.path.exists(self.output_dir):
            os.mkdir(self.output_dir)
        dir_idx = 0
        while os.path.exists(os.path.join(self.output_dir, f"dataset_{dir_idx}")):
            dir_idx+=1
        current_dataset_path = os.path.join(self.output_dir, f"dataset_{dir_idx}")
        os.mkdir(current_dataset_path)
        self.get_logger().info(f"store image to: {current_dataset_path}")
        self.current_dataset_path = current_dataset_path
        
        dir_color = os.path.join(self.current_dataset_path,"color")
        dir_depth = os.path.join(self.current_dataset_path,"depth")
        if not os.path.exists(dir_color):
            os.mkdir(dir_color)
        if not os.path.exists(dir_depth):
            os.mkdir(dir_depth)

        self.dir_color = dir_color
        self.dir_depth = dir_depth
        # camera info topic for the camera calibration parameters
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )
        # output dir
        
        # select the type of input to use for the pose estimation
        if (bool(self.use_depth_input)):
            # use both rgb and depth image topics for the pose estimation

            # create a message filter to synchronize the image and depth image topics
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)

            # create synchronizer between the 2 topics using message filters and approximate time policy
            # slop is the maximum time difference between messages that are considered synchronized
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            # rely only on the rgb image topic for the pose estimation

            # create a subscription to the image topic
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # Set up publishers
       
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # code for updated version of cv2 (4.7.0)
        
        # old code version
        # self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        # self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        # get the intrinsic matrix and distortion coefficients from the camera info
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().info("Camera info received.")
        self.get_logger().info("Intrinsic matrix: {}".format(self.intrinsic_mat))
        self.get_logger().info("Distortion coefficients: {}".format(self.distortion))
        self.get_logger().info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))

        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        self.get_logger().info("rgb only")
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        # convert the image messages to cv2 format
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")

    def depth_image_callback(self, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.img_idx >= 150:
            self.get_logger().info('Received 150 images, shutting down...')
            rclpy.shutdown()

        self.get_logger().warn("rgb and depth")
        # convert the image messages to cv2 format
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        print(f"store image idx = {self.img_idx}")
        path_color = os.path.join(self.dir_color,f"{self.img_idx}_color.png")
        path_depth = os.path.join(self.dir_depth,f"{self.img_idx}_depth.png")
        cv2.imwrite(path_color, cv_image)
        cv2.imwrite(path_depth, cv_depth_image)
        self.img_idx = self.img_idx+1


    def initialize_parameters(self):
        self.declare_parameter(
            name="output_dir",
            value="./",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="output path to store the data",
            ),
        )
        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/camera/aligned_depth_to_color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera/aligned_depth_to_color/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        # read parameters from aruco_params.yaml and store them
        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        self.output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [6131]
# Author List:      [Agrim Chauhan, Saksham Kaushal, Rayhan Khan, Manan Sood]
# Filename:         task1A.py
# Task:             1A - StrataCobot (eYRC 2026-27)
# Functions:
#                   detect_ores, __init__, depthimagecb, colorimagecb, caminfocb, process_image,
#                   main
# Nodes:
#                   Name - [ ore_tf_publisher ]
#                   Publishing Topics - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                       /camera/camera/aligned_depth_to_color/image_raw,
#                       /camera/camera/color/camera_info, /tf, /tf_static ]

################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import CameraInfo, Image


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    center_ore_list = []
    ore_type_list = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    color_ranges = {
        'azurite_ore': (
            np.array([95, 150, 80]),
            np.array([115, 255, 255])
        ),
        'malachite_ore': (
            np.array([65, 150, 100]),
            np.array([85, 255, 255])
        ),
        'vanadinite_ore': (
            np.array([5, 230, 100]),
            np.array([15, 255, 255])
        )
    }

    for ore_type, (lower, upper) in color_ranges.items():

        # Create a binary mask for this ore colour
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological operations to clean the mask
        kernel = np.ones((3, 3), np.uint8)

        # Remove small isolated noise
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        # Fill small holes
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # Find separate regions in the mask
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:

            # Ignore very small regions
            area = cv2.contourArea(contour)

            if area < 50:
                continue
            #Get the bound rectangle of the ore
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate contour centroid
            M = cv2.moments(contour)

            if M['m00'] == 0:
                continue

            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])

            # Draw boundary around the ore
            cv2.rectangle(
                image,
                (x, y),
                (x + w, y + h),
                (0, 0, 0),
                2
            )
            # Store the centre pixel and ore type
            center_ore_list.append((cX, cY))
            ore_type_list.append(ore_type)

            # Draw detection on the image
            cv2.circle(
                image,
                (cX, cY),
                5,
                (0, 0, 0),
                -1
            )

    return center_ore_list, ore_type_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None                                                            # camera intrinsics variable (from caminfocb())
        self.depth_frame=None  #Depth image frame name

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        # Stores the pixel position associated with each ore ID
        self.ore_tracks = {
            'azurite_ore': {},
            'malachite_ore': {},
            'vanadinite_ore': {}
        }


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''


        self.depth_image = self.bridge.imgmsg_to_cv2(
            data,
            desired_encoding='passthrough'
        )
        self.depth_frame = data.header.frame_id


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''


        self.cv_image = self.bridge.imgmsg_to_cv2(
            data,
            desired_encoding='bgr8'
        )


    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''


        self.fx = float(data.k[0])
        self.fy = float(data.k[4])
        self.cx = float(data.k[2])
        self.cy = float(data.k[5])


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''


        # Make sure all required camera data has arrived
        if (
            self.cv_image is None
            or self.depth_image is None
            or self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
            or self.depth_frame is None
        ):
            return

        # Detect all ores in the current colour image
        center_ore_list, ore_type_list = detect_ores(self.cv_image)

        # Get the transform from the camera optical frame to base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame,
                self.depth_frame,
                rclpy.time.Time()
            )
        except Exception:
            return

        # Assign stable IDs to the detected ores
        detections_by_type = {}

        for position, ore_type in zip(center_ore_list, ore_type_list):
            if ore_type not in detections_by_type:
                detections_by_type[ore_type] = []

            detections_by_type[ore_type].append(position)

        for ore_type, detections in detections_by_type.items():

            # First frame for this ore type
            if len(self.ore_tracks[ore_type]) == 0:

                # Sort from left to right.
                # Leftmost becomes ID 1.
                detections.sort(key=lambda p: p[0])

                for i, position in enumerate(detections):
                    self.ore_tracks[ore_type][i + 1] = position

            # Subsequent frames
            elif len(detections) == 2:

                old_1 = self.ore_tracks[ore_type][1]
                old_2 = self.ore_tracks[ore_type][2]

                # Two possible assignments:
                #
                # A: detection 0 -> ID 1
                #    detection 1 -> ID 2
                #
                # B: detection 0 -> ID 2
                #    detection 1 -> ID 1

                distance_a = (
                    (detections[0][0] - old_1[0]) ** 2 +
                    (detections[0][1] - old_1[1]) ** 2 +
                    (detections[1][0] - old_2[0]) ** 2 +
                    (detections[1][1] - old_2[1]) ** 2
                )

                distance_b = (
                    (detections[0][0] - old_2[0]) ** 2 +
                    (detections[0][1] - old_2[1]) ** 2 +
                    (detections[1][0] - old_1[0]) ** 2 +
                    (detections[1][1] - old_1[1]) ** 2
                )

                if distance_a <= distance_b:
                    self.ore_tracks[ore_type][1] = detections[0]
                    self.ore_tracks[ore_type][2] = detections[1]
                else:
                    self.ore_tracks[ore_type][1] = detections[1]
                    self.ore_tracks[ore_type][2] = detections[0]

        # Process every detected ore
        for (cX, cY), ore_type in zip(center_ore_list, ore_type_list):
            if self.ore_tracks[ore_type][1] == (cX, cY):
                ore_id = 1
            else:
                ore_id = 2
            # Make sure the pixel is inside the depth image
            if (
                cX < 0
                or cX >= self.depth_image.shape[1]
                or cY < 0
                or cY >= self.depth_image.shape[0]
            ):
                continue

            # Take a small depth window around the centre pixel
            y1 = max(0, cY - 4)
            y2 = min(self.depth_image.shape[0], cY + 5)
            x1 = max(0, cX - 4)
            x2 = min(self.depth_image.shape[1], cX + 5)

            patch = self.depth_image[y1:y2, x1:x2]

            # Keep only valid depth measurements
            valid_depth = patch[
                np.isfinite(patch) & (patch > 0.0)
            ]

            if valid_depth.size == 0:
                continue

            # Median is more robust than using a single pixel
            z = float(np.median(valid_depth))

            # Convert pixel coordinates into camera-frame coordinates
            x = float((cX - self.cx) * z / self.fx)
            y = float((cY - self.cy) * z / self.fy)

            # Create a point in the camera optical frame
            point_camera = PointStamped()

            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.header.frame_id = self.depth_frame

            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # Transform the point into base_link
            try:
                point_base = do_transform_point(
                    point_camera,
                    tf
                )
            except Exception:
                continue

            # Create the transform
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = base_frame
            t.child_frame_id = f'{ore_type}_{ore_id}'

            t.transform.translation.x = float(point_base.point.x)
            t.transform.translation.y = float(point_base.point.y)
            t.transform.translation.z = float(point_base.point.z)

            # Required orientation: identity quaternion
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transform
            self.br.sendTransform(t)

            # Draw the detection
            cv2.circle(
                self.cv_image,
                (cX, cY),
                6,
                (0, 0, 0),
                2
            )

            cv2.putText(
                self.cv_image,
                f'{ore_type}_{ore_id}',
                (cX + 8, cY),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

            # Save the annotated camera image
            cv2.imwrite(
            'SC#6131_task1A_detection.png',
            self.cv_image
            )

            # Display the annotated camera image
            cv2.imshow('Task 1A - Ore Detection', self.cv_image)
            cv2.waitKey(1)



##################### FUNCTION DEFINITION #######################

def main():
    rclpy.init(args=sys.argv)

    ore_tf_class = ore_tf()

    rclpy.spin(ore_tf_class)

    ore_tf_class.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()                                             # shutdown process


if __name__ == '__main__':

    main()

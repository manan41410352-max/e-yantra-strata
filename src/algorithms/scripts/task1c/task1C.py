#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1C of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1c_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data
)
from sensor_msgs.msg import LaserScan


##################### TASK CONSTANTS #######################

# The route is published here ONCE, before your node starts, and held for late
# subscribers. An ordinary subscription waits forever for a message already sent.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'
map_topic = '/map'
# What /cmd_vel accepts. The base clips anything beyond these, and clipping one of the
# two changes their ratio - which is the arc the base actually drives.
max_linear_mps = 0.5
max_angular_rps = 1.0


##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the route
                    published on /ebot_path, through its waypoints, in order.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # ->  NOTE: The subscription to 'path_topic' below uses the default settings and will
        #           NEVER receive the route - see the INSTRUCTIONS in 'pathcb' and fix it there.
        path_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.pathcb,
            path_qos
        )        
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.mapcb,
            map_qos
        )

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        self.current_waypoint = 0
        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())
        
        ############ ADD YOUR CODE HERE ############
        self.map = None 
        ############################################


    def pathcb(self, data):
        '''
        Description:    Callback function for the route topic.
                        Use this function to receive the waypoints the eBot has to drive.

        Args:
            data (Path):    The route, as a sequence of poses

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if self.route is not None:
            return

        self.route = []

        for pose in data.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            self.route.append((x, y))

        self.current_waypoint = 0

        self.get_logger().info(
            f'Received route with {len(self.route)} waypoints '
            f'in frame {data.header.frame_id}'
        )
        ############################################


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        x = position.x
        y = position.y

        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        self.odom = (x, y, yaw)

        ############################################


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic.
                        Use this function to receive what the lidar currently sees.

        Args:
            data (LaserScan):    One lidar sweep

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        self.scan = data

        ############################################

    def mapcb(self, data):
        '''
        Receive the occupancy grid containing the rock layout.
        '''
        if self.map is not None:
            return
        self.map = data
        self.get_logger().info(
            f'Received map: {data.info.width} x {data.info.height}, '
            f'resolution {data.info.resolution:.3f} m'
       )
        
    def world_to_map(self, x, y):
        '''
        Convert world coordinates to occupancy-grid cell coordinates.
        '''

        origin = self.map.info.origin.position
        resolution = self.map.info.resolution

        col = int((x - origin.x) / resolution)
        row = int((y - origin.y) / resolution)

        return row, col

    def map_occupied(self, x, y):
        '''
        Return True if the map says the given world position is occupied.
        '''

        row, col = self.world_to_map(x, y)

        width = self.map.info.width
        height = self.map.info.height

        if row < 0 or row >= height or col < 0 or col >= width:
            return True

        index = row * width + col

        return self.map.data[index] >= 50

    def get_obstacle_info(self):
        '''
        Check for obstacles in front of the eBot and determine
        which side has more free space.
        '''

        if self.scan is None:
            return float('inf'), 0.0, 0.0

        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for i, r in enumerate(self.scan.ranges):

            if not math.isfinite(r):
                continue

            if r < self.scan.range_min or r > self.scan.range_max:
                continue

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            # Front sector: approximately ±30 degrees
            if abs(angle) <= math.radians(30):
                front_min = min(front_min, r)

            # Left sector
            elif math.radians(30) < angle <= math.radians(90):
                left_min = min(left_min, r)

            # Right sector
            elif -math.radians(90) <= angle < -math.radians(30):
                right_min = min(right_min, r)

        return front_min, left_min, right_min
    
    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # Wait until all required sensor data has arrived.
        if self.route is None:
            return

        if self.odom is None:
            return

        if self.scan is None:
            return
        
        # Check whether all waypoints have been completed.
        if self.current_waypoint >= len(self.route):
            self.stop_rover()
            return

        # Current robot pose.
        x, y, yaw = self.odom
        front_min, left_min, right_min = self.get_obstacle_info()

        # Current target waypoint.
        tx, ty = self.route[self.current_waypoint]

        # Position error.
        dx = tx - x
        dy = ty - y

        distance = math.hypot(dx, dy)

        # --------------------------------------------------
        # Check whether the current waypoint is reached.
        # --------------------------------------------------

        waypoint_tolerance = 0.20

        if distance < waypoint_tolerance:

            self.get_logger().info(
                f'Reached waypoint '
                f'{self.current_waypoint + 1}/{len(self.route)}'
            )

            self.current_waypoint += 1

            # Last waypoint reached.
            if self.current_waypoint >= len(self.route):
                self.stop_rover()

                self.get_logger().info(
                    'Route complete. Rover stopped.'
                )

                return

            # Get the next waypoint.
            tx, ty = self.route[self.current_waypoint]

            dx = tx - x
            dy = ty - y

            distance = math.hypot(dx, dy)

        # --------------------------------------------------
        # Obstacle avoidance
        # --------------------------------------------------

        obstacle_distance = 0.65

        if front_min < obstacle_distance:

            cmd = Twist()

            # Turn toward the side with more free space.
            if left_min > right_min:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.6

            self.cmd_pub.publish(cmd)

            return

        # --------------------------------------------------
        # Calculate desired heading.
        # --------------------------------------------------

        desired_heading = math.atan2(dy, dx)

        heading_error = math.atan2(
            math.sin(desired_heading - yaw),
            math.cos(desired_heading - yaw)
        )

        # --------------------------------------------------
        # Angular velocity.
        # --------------------------------------------------

        Kp_angular = 1.5

        angular = Kp_angular * heading_error

        angular = max(
            -max_angular_rps,
            min(max_angular_rps, angular)
        )

        # --------------------------------------------------
        # Linear velocity.
        # --------------------------------------------------

        Kp_linear = 0.5

        # Slow down when the robot is facing away from
        # the waypoint.
        heading_factor = max(
            0.0,
            math.cos(heading_error)
        )

        linear = Kp_linear * distance * heading_factor

        linear = max(
            0.0,
            min(max_linear_mps, linear)
        )

        # --------------------------------------------------
        # Publish command.
        # --------------------------------------------------

        cmd = Twist()

        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

        ############################################

    def stop_rover(self):
        '''
        Stop the eBot by publishing zero velocity.
        '''

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_nav_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot navigation process') # logging information

    ebot_nav_class = ebot_nav()                                     # creating a new object for class 'ebot_nav'

    rclpy.spin(ebot_nav_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_nav_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()

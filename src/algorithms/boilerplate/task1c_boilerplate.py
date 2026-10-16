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
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


##################### TASK CONSTANTS #######################

# The route is published here ONCE, before your node starts, and held for late
# subscribers. An ordinary subscription waits forever for a message already sent.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'

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
        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your controller needs to keep between cycles.
        #       ->  HINT: How far along the route you have got, for a start.

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

        # INSTRUCTIONS & HELP :

        #	->  This callback will not fire as written. The route is published once, before
        #       this node starts, and held for late subscribers, so the subscription has to
        #       match the publisher's QoS-
        #           QoSProfile(depth=1, history=..., reliability=..., durability=...)
        #       ->  HINT: 'ros2 topic info /ebot_path -v' says what the publisher offers. The
        #                 policies are in 'rclpy.qos'; a held message is TRANSIENT_LOCAL.

        #   ->  Store the waypoints, and log how many arrived and in which frame.
        #       ->  HINT: data.poses[i].pose.position

        #   ->  Take the route once. Replacing it mid-run resets whatever you keep with it.

        ############################################


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the position and the heading of the base.
        #       ->  HINT: data.pose.pose.position, data.pose.pose.orientation (a quaternion)
        #                     yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        #                 or 'euler_from_quaternion' from tf_transformations.

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

        # INSTRUCTIONS & HELP :

        #	->  Store the scan. Read the message definition first
        #       -> https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html

        #   ->  Work out the direction of each beam-
        #           angle_i = angle_min + i * angle_increment      # 0 is straight ahead

        #   ->  Keep only the entries that are measurements-
        #           math.isfinite(r) and range_min <= r <= range_max

        #   ->  The ranges are measured from the LIDAR, not from the centre of the base.
        #       ->  HINT: /tf carries where it is mounted. Do not type in an offset.

        ############################################


    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Return early until route, odometry and scan have all arrived.

        #   ->  Steer towards the waypoint you are heading for and publish a Twist on
        #       self.cmd_pub. Only linear.x and angular.z do anything-
        #           distance = hypot(tx - x, ty - y)
        #           heading  = wrap(atan2(ty - y, tx - x) - yaw)
        #           angular  = Kp * heading
        #       ->  HINT: wrap(e) is atan2(sin(e), cos(e)); without it the base turns the
        #                 long way round. Drive slower the worse the heading error is.

        #   ->  Keep the command inside the limits above yourself.

        #   ->  Decide when a waypoint is done and the next becomes the target. Too early and
        #       you never went there; too late and you have already driven on.
        #       ->  NOTE: The waypoints must be taken IN THE ORDER the route lists them.

        #   ->  Get around the props. A controller that follows the line and ignores the scan
        #       will not get through.
        #       ->  HINT: Somewhere to start: take the nearest valid range in a cone ahead,
        #                 and compare the room on each side to pick a way round.
        #       ->  NOTE: The eBot's geometry is in its URDF, and its widest part is not its
        #                 chassis.

        #   ->  This is a four-wheel SKID STEER. How cleanly it turns on the spot is
        #       something to measure, not to assume.

        #   ->  Check what the base did on /odom, not on the command you sent.

        #   ->  Stop the base once the route is done. Think about what "done" means for a
        #       base that sails past the last waypoint without ever being close to it.

        #   ->  Nothing here should raise. An uncaught exception kills the node and leaves
        #       the base coasting.

        ############################################


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

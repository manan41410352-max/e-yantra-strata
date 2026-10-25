#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:         task1B.py
# Task:             1B - StrataCobot (eYRC 2026-27)
# Functions:
#                   __init__, tcpposecb, jointstatecb, armstatuscb, switch_controller,
#                   switch_finished, publish_twist, publish_joint, process_waypoints, main
# Nodes:
#                   Name - [ arm_waypoints_node ]
#                   Publishing Topics - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status, /clock ]
#                   Service Clients - [ /ur_arm_controller/switch_controller ]

################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.10     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15

# Position P-controller
Kp = 0.5

# Arm status
status_first_critical = 20

# Motion phases
PH_WAIT_READY = 0
PH_SWITCH_TO_JOINT = 1
PH_UNFOLD = 2
PH_SETTLE = 3
PH_SWITCH_TO_TWIST = 4
PH_WAYPOINTS = 5
PH_DONE = 6
PH_FAULT = 7


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')
        self.timer = self.create_timer(control_rate, self.process_waypoints)            # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())

        self.current_waypoint = 0
        self.holding = False
        self.hold_start_time = None

        # Joint-space unfolding
        self.unfold_pose = [
            -2.0216,
            -2.1366,
            -1.5538,
            -1.0221,
            1.5708,
            1.5708
        ]

        self.unfold_speed_rps = 0.25
        self.unfold_gain = 2.0
        self.unfold_tol_rad = 0.01
        self.settle_time_s = 1.0

        self.phase = PH_WAIT_READY
        self.phase_start_time = None
        self.switch_future = None                                                   # activating the twist controller to start with






    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        self.tcp_pose = data.pose


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''



        self.joint_angles = [
            data.position[data.name.index(j)]
            for j in joint_names
        ]


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        self.arm_status = data.data


    def switch_controller(self, controller):
        '''
        Description:    Function to make one of the arm's two command interfaces the active
                        one, so that commands published to it are acted on.

        Args:
            controller  (str):      Name of the controller to activate, either
                                    'twist_controller' or 'joint_controller'

        Returns:
            success     (bool):     Whether the controller was activated
        '''


        if not self.switch_cli.service_is_ready():
            return False

        req = SwitchController.Request()

        req.activate_controllers = [controller]

        if controller == twist_controller:
            req.deactivate_controllers = [joint_controller]
        else:
            req.deactivate_controllers = [twist_controller]

        req.strictness = SwitchController.Request.STRICT

        self.switch_future = self.switch_cli.call_async(req)

        return True

    def switch_finished(self):

        if self.switch_future is None:
            return None

        if not self.switch_future.done():
            return None

        response = self.switch_future.result()

        self.switch_future = None

        if response is None:
            return False

        return bool(response.ok)

    def publish_twist(self, vx, vy, vz):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)


    def publish_joint(self, velocities):

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(joint_names)
        msg.velocities = [float(v) for v in velocities]

        self.joint_pub.publish(msg)


    def process_waypoints(self):
        '''
        Description:    Timer function used to drive the tool through the waypoints.

        Args:
        Returns:
        '''


        # Wait until all required feedback is available
        if self.tcp_pose is None:
            return

        if self.joint_angles is None:
            return

        if self.arm_status is None:
            return


        # ---------------------------------------------------------
        # CRITICAL FAULT
        # ---------------------------------------------------------

        if self.arm_status >= status_first_critical:

            self.phase = 7

            self.publish_twist(
                0.0, 0.0, 0.0
            )

            self.publish_joint(
                [0.0] * 6
            )

            self.get_logger().error(
                f'Critical arm status: {self.arm_status}'
            )

            return


        # ---------------------------------------------------------
        # PHASE 0: WAIT FOR CONTROLLER SERVICE
        # ---------------------------------------------------------

        if self.phase == 0:

            if self.switch_cli.service_is_ready():

                self.get_logger().info(
                    'Switching to joint controller'
                )

                self.switch_controller(joint_controller)

                self.phase = 1

            return


        # ---------------------------------------------------------
        # PHASE 1: WAIT FOR JOINT CONTROLLER
        # ---------------------------------------------------------

        if self.phase == 1:

            result = self.switch_finished()

            if result is None:
                return

            if not result:

                self.get_logger().error(
                    'Failed to switch to joint controller'
                )

                self.phase = 7
                return

            self.get_logger().info(
                'Joint controller active. Unfolding arm.'
            )

            self.phase = 2

            return


        # ---------------------------------------------------------
        # PHASE 2: UNFOLD ARM IN JOINT SPACE
        # ---------------------------------------------------------

        if self.phase == 2:

            errors = [
                target - current
                for target, current
                in zip(self.unfold_pose, self.joint_angles)
            ]

            biggest_error = max(
                abs(error)
                for error in errors
            )

            # Unfolding complete
            if biggest_error < self.unfold_tol_rad:

                self.publish_joint([0.0] * 6)

                self.phase_start_time = (
                    self.get_clock().now()
                )

                self.get_logger().info(
                    'Arm unfolded. Settling.'
                )

                self.phase = 3

                return


            # Proportional joint velocity
            speed = min(
                self.unfold_speed_rps,
                self.unfold_gain * biggest_error
            )

            # Stay below joint velocity limit
            speed = min(
                speed,
                0.9 * cap_joint_rps
            )

            joint_velocities = [
                (error / biggest_error) * speed
                for error in errors
            ]

            self.publish_joint(
                joint_velocities
            )

            return


        # ---------------------------------------------------------
        # PHASE 3: SETTLE AFTER UNFOLD
        # ---------------------------------------------------------

        if self.phase == 3:

            self.publish_joint([0.0] * 6)

            elapsed = (
                self.get_clock().now()
                - self.phase_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.settle_time_s:

                self.get_logger().info(
                    'Switching to twist controller'
                )

                self.switch_controller(
                    twist_controller
                )

                self.phase = 4

            return


        # ---------------------------------------------------------
        # PHASE 4: WAIT FOR TWIST CONTROLLER
        # ---------------------------------------------------------

        if self.phase == 4:

            result = self.switch_finished()

            if result is None:
                return

            if not result:

                self.get_logger().error(
                    'Failed to switch to twist controller'
                )

                self.phase = 7
                return

            self.get_logger().info(
                'Twist controller active. Starting waypoints.'
            )

            self.phase = 5

            return


        # ---------------------------------------------------------
        # PHASE 5: ORIGINAL WAYPOINT P CONTROLLER
        # ---------------------------------------------------------

        if self.phase == 5:

            # All waypoints completed
            if self.current_waypoint >= len(waypoints):

                self.publish_twist(
                    0.0, 0.0, 0.0
                )

                self.phase = 6

                self.get_logger().info(
                    'All waypoints completed.'
                )

                return


            current_x = self.tcp_pose.position.x
            current_y = self.tcp_pose.position.y
            current_z = self.tcp_pose.position.z

            target_x, target_y, target_z = (
                waypoints[self.current_waypoint]
            )


            # Position error
            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = target_z - current_z


            distance = math.sqrt(
                error_x ** 2 +
                error_y ** 2 +
                error_z ** 2
            )


            # -----------------------------------------------------
            # WAYPOINT REACHED
            # -----------------------------------------------------

            if distance < 0.005:

                if not self.holding:

                    self.holding = True

                    self.hold_start_time = (
                        self.get_clock().now()
                    )

                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint + 1} reached. Holding.'
                    )


                elapsed = (
                    self.get_clock().now()
                    - self.hold_start_time
                ).nanoseconds / 1e9


                # Keep publishing ZERO during hold
                self.publish_twist(
                    0.0, 0.0, 0.0
                )


                # Hold for required 2 seconds
                if elapsed >= 2.0:

                    self.current_waypoint += 1

                    self.holding = False
                    self.hold_start_time = None

                    self.get_logger().info(
                        f'Moving to waypoint {self.current_waypoint + 1}'
                    )

                return


            # -----------------------------------------------------
            # MOVE TOWARD WAYPOINT
            # -----------------------------------------------------

            self.holding = False
            self.hold_start_time = None


            # Original P controller

            vx = Kp * error_x
            vy = Kp * error_y
            vz = Kp * error_z


            # Magnitude of commanded velocity
            speed = math.sqrt(
                vx ** 2 +
                vy ** 2 +
                vz ** 2
            )


            # Cap the WHOLE velocity vector
            if speed > cap_linear_mps:

                scale = cap_linear_mps / speed

                vx *= scale
                vy *= scale
                vz *= scale


            # No orientation control
            self.publish_twist(
                vx, vy, vz
            )

            return


        # ---------------------------------------------------------
        # PHASE 6: DONE
        # ---------------------------------------------------------

        if self.phase == 6:

            self.publish_twist(
                0.0, 0.0, 0.0
            )

            return


        # ---------------------------------------------------------
        # PHASE 7: FAULT
        # ---------------------------------------------------------

        if self.phase == 7:

            self.publish_twist(
                0.0, 0.0, 0.0
            )

            self.publish_joint(
                [0.0] * 6
            )

            return

    def publish_twist(self, vx, vy, vz):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)


    def publish_joint(self, velocities):

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.joint_names = joint_names
        msg.velocities = velocities

        self.joint_pub.publish(msg)
##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node

    node.get_logger().info('Node created: Arm waypoints process')   # logging information

    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'

    rclpy.spin(arm_waypoints_class)                                 # spining on the object to make it alive in ROS 2 DDS

    arm_waypoints_class.destroy_node()                              # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()

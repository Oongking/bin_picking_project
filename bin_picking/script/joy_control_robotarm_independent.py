#!/usr/bin/env python

"""https://fzi-robot-interface-proposal.readthedocs.io/en/latest/industry/UR.html"""
""" https://answers.ros.org/question/391998/how-to-call-the-switch-controller-service-correctly/ """
# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

# msg & convert
import geometry_msgs.msg 
from geometry_msgs.msg import TwistStamped 
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray,String

# Utility
import numpy as np
import time
import copy
import sys


class JoyStatus:
    def __init__(self):
        self.center = False
        self.select = False
        self.start = False
        self.L3 = False
        self.R3 = False
        self.square = False
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.triangle = False
        self.cross = False
        self.circle = False
        self.L1 = False
        self.R1 = False
        self.L2 = False
        self.R2 = False
        self.left_analog_x = 0.0
        self.left_analog_y = 0.0
        self.right_analog_x = 0.0
        self.right_analog_y = 0.0

class NubwoStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)

        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.L3 = msg.buttons[10] == 1
        self.R3 = msg.buttons[11] == 1
        self.square = msg.buttons[3] == 1
        self.circle = msg.buttons[1] == 1
        self.up = msg.axes[6] > 0.1
        self.down = msg.axes[6] < -0.1
        self.left = msg.axes[5] > 0.1
        self.right = msg.axes[5] < -0.1
        self.triangle = msg.buttons[0] == 1
        self.cross = msg.buttons[2] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg

class XBoxStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        self.center = msg.buttons[8] == 1
        self.select = msg.buttons[6] == 1
        self.start = msg.buttons[7] == 1
        self.L3 = msg.buttons[9] == 1
        self.R3 = msg.buttons[10] == 1
        self.square = msg.buttons[2] == 1
        self.circle = msg.buttons[1] == 1
        self.up = msg.axes[7] > 0.1
        self.down = msg.axes[7] < -0.1
        self.left = msg.axes[6] > 0.1
        self.right = msg.axes[6] < -0.1
        self.triangle = msg.buttons[3] == 1
        self.cross = msg.buttons[0] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.axes[2] < -0.5
        self.R2 = msg.axes[5] < -0.5
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg

class PS3DualShockStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.cross = msg.buttons[0] == 1
        self.circle = msg.buttons[1] == 1
        self.triangle = msg.buttons[2] == 1
        self.square = msg.buttons[3] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.center = msg.buttons[10] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]

        self.orig_msg = msg

class PS3Status(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.center = msg.buttons[16] == 1
        self.select = msg.buttons[0] == 1
        self.start = msg.buttons[3] == 1
        self.L3 = msg.buttons[1] == 1
        self.R3 = msg.buttons[2] == 1
        self.square = msg.axes[15] < 0
        self.up = msg.axes[4] < 0
        self.down = msg.axes[6] < 0
        self.left = msg.axes[7] < 0
        self.right = msg.axes[5] < 0
        self.triangle = msg.axes[12] < 0
        self.cross = msg.axes[14] < 0
        self.circle = msg.axes[13] < 0
        self.L1 = msg.axes[10] < 0
        self.R1 = msg.axes[11] < 0
        self.L2 = msg.axes[8] < 0
        self.R2 = msg.axes[9] < 0
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg

class PS3WiredStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.center = msg.buttons[16] == 1
        self.select = msg.buttons[0] == 1
        self.start = msg.buttons[3] == 1
        self.L3 = msg.buttons[1] == 1
        self.R3 = msg.buttons[2] == 1
        self.square = msg.buttons[15] == 1
        self.up = msg.buttons[4] == 1
        self.down = msg.buttons[6] == 1
        self.left = msg.buttons[7] == 1
        self.right = msg.buttons[5] == 1
        self.triangle = msg.buttons[12] == 1
        self.cross = msg.buttons[14] == 1
        self.circle = msg.buttons[13] == 1
        self.L1 = msg.buttons[10] == 1
        self.R1 = msg.buttons[11] == 1
        self.L2 = msg.buttons[8] == 1
        self.R2 = msg.buttons[9] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg

class PS4Status(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msg/Joy
        self.center = msg.buttons[12] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.L3 = msg.buttons[10] == 1
        self.R3 = msg.buttons[11] == 1
        self.square = msg.buttons[0] == 1
        self.up = msg.axes[10] < 0
        self.down = msg.axes[10] > 0
        self.left = msg.axes[9] < 0
        self.right = msg.axes[9] > 0
        self.triangle = msg.buttons[3] == 1
        self.cross = msg.buttons[1] == 1
        self.circle = msg.buttons[2] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[5]
        self.right_analog_y = msg.axes[2]
        self.orig_msg = msg

class PS4WiredStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msg/Joy
        self.center = msg.buttons[10] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.L3 = msg.buttons[11] == 1
        self.R3 = msg.buttons[12] == 1
        self.square = msg.buttons[3] == 1
        self.up = msg.axes[7] < 0
        self.down = msg.axes[7] > 0
        self.left = msg.axes[6] < 0
        self.right = msg.axes[6] > 0
        self.triangle = msg.buttons[2] == 1
        self.cross = msg.buttons[0] == 1
        self.circle = msg.buttons[1] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg


class ServoJoy:
    def __init__(self):
        self.prev_time = rospy.Time.now()
        self.InvertmodeTime = rospy.Time.now()

        self.pub = rospy.Publisher("/servo_server/delta_twist_cmds", TwistStamped, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                    # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                                # outgoing message queue used for asynchronous publishing
        self.pub2Ur = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        self.rate = rospy.Rate(100) # 10hz
        self.subjoint = rospy.Subscriber("/joint_group_position_controller/command", Float64MultiArray, self.sent2Ur, queue_size=1)
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)
        self.Invertmove = False

    def sent2Ur(self, msg):

        goal = f"movel([{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]},{msg.data[4]},{msg.data[5]}], a=5, v=0.25, t=0, r=0)"
        # print(goal)
        self.pub2Ur.publish(goal)

    def joyCB(self, msg):
        axes_amount = len(msg.axes)
        buttons_amount = len(msg.buttons)
        if axes_amount == 27 and buttons_amount == 19:
            status = PS3WiredStatus(msg)
        elif axes_amount == 8 and buttons_amount == 11:
            status = XBoxStatus(msg)
        elif axes_amount == 20 and buttons_amount == 17:
            status = PS3Status(msg)
        elif axes_amount == 14 and buttons_amount == 14:
            status = PS4Status(msg)
        elif axes_amount == 8 and buttons_amount == 13:
            status = PS4WiredStatus(msg)
        elif axes_amount == 6 and buttons_amount == 17:
            status = PS3DualShockStatus(msg)
        elif axes_amount == 7 and buttons_amount == 12:
            status = NubwoStatus(msg)
        else:
            raise Exception(
                "Unknown joystick, axes: {}, buttons: {}".format(
                    axes_amount, buttons_amount
                )
            )
        self.run(status)

    def computeMoveFromJoy(self, status):
        move = TwistStamped() # Creates a Twist message type object
        now = rospy.get_rostime()
        # move.header.frame_id = gps_frame_id
        move.header.stamp.secs = now.secs
        move.header.stamp.nsecs = now.nsecs

        move_speed_pos = 0.5
        rotate_speed_pos = 1.0
        # x
        if status.up:
            xmove = move_speed_pos
        elif status.down:
            xmove = -move_speed_pos
        else:
            xmove = 0.0 
        # y
        if status.right:
            ymove = -move_speed_pos
        elif status.left:
            ymove = move_speed_pos
        else:
            ymove = 0.0
        # z
        if status.L2:
            zmove = move_speed_pos
        elif status.R2:
            zmove = -move_speed_pos
        else:
            zmove = 0.0
        
        # roll
        roll = status.left_analog_x * rotate_speed_pos
        # pitch
        pitch = status.left_analog_y * rotate_speed_pos
        # yaw
        yaw = status.right_analog_x * rotate_speed_pos

        move.twist.linear.x = xmove
        move.twist.linear.y = ymove
        move.twist.linear.z = zmove
        move.twist.angular.x = roll
        move.twist.angular.y = pitch
        move.twist.angular.z = yaw

        if status.circle:
            diff_matrix = tf.transformations.euler_matrix(roll, pitch, yaw, 'sxyz')
            print(diff_matrix)

        return move


    def run(self, status):

        move = self.computeMoveFromJoy(status)

        now = rospy.Time.from_sec(time.time())

        if status.triangle == 1:
            if (now - self.InvertmodeTime).to_sec() > 0.5:
                self.Invertmove = not self.Invertmove
            self.InvertmodeTime = now
            print(f"state {self.Invertmove}")
            if not self.Invertmove:
                rospy.loginfo(" Cartesian Mode ")
            else:
                rospy.loginfo(" Invert Mode ")

        # placement.time_from_start = now - self.prev_time
        if (now - self.prev_time).to_sec() > 1 / 100.0:
            # rospy.loginfo(new_pose)
            self.pub.publish(move)
            self.prev_time = now
            

if __name__ == "__main__":
    rospy.init_node('joy_servo') # Initializes a node
    app = ServoJoy()
    rospy.spin()
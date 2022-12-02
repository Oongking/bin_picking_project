#! /usr/bin/env python

from turtle import pos
import rospy
import sys
import actionlib
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import time
moveit_commander.roscpp_initialize(sys.argv)



#blue #yellow  #red  #green 
waypoints = [[0.55,0.8,0.707,-0.707],[0.55,-0.2,0.707,-0.707],[-0.2,-0.55,1,0],[-0.55,0.2,0.707,0.707]]
placepoint = [[-0.05,1.7,0.707,0.707],[1.7,0.05,0,1],[0.05,-1.7,0.707,-0.707],[-1.7,-0.05,1,0]]
prepoint = [[-1.5,1.5,0,1]]

MotionPlan = {
            'prepick' : [-1.5707, 0, -1.5707, 0, 1.5707, 0],
            'pick' : [-1.5707, -1.5794, -0.4339, 0.4339, 1.5707, 0],
            'open' : [0,0],
            'close' : [0.04, 0.04]
}
bridge = CvBridge()
colordict = {
        'Blue': [[110, 0, 0], [125, 255, 255]],
        'Green': [[50, 255, 0],[60, 255, 255]],
        'Yellow': [[15, 0, 0],[30, 255, 255]],
        'Red': [[1, 0, 0],[13, 255, 255]]
    }

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group_arm = moveit_commander.MoveGroupCommander(group_name)

group_name = "gripper"
group_gripper = moveit_commander.MoveGroupCommander(group_name)

def controlGripper(pose):
    print(pose)
    joint_goal = group_gripper.get_current_joint_values()
    joint_goal[0] = MotionPlan[pose][0]
    joint_goal[1] = MotionPlan[pose][1]

    group_gripper.go(joint_goal,wait=True)
    group_gripper.stop()

def controlArm(pose):
    print(pose)
    joint_goal = group_arm.get_current_joint_values()
    joint_goal[0] = MotionPlan[pose][0]
    joint_goal[1] = MotionPlan[pose][1]
    joint_goal[2] = MotionPlan[pose][2]
    joint_goal[3] = MotionPlan[pose][3]
    joint_goal[4] = MotionPlan[pose][4]
    joint_goal[5] = MotionPlan[pose][5]

    group_arm.go(joint_goal,wait=True)
    group_arm.stop()


def move_base_client(x,y,qz,qw):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now();
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    client.send_goal(goal)
    wait = client.wait_for_result()

    return client.get_result()


def callback(data):
    global BlueError
    global GreenError
    global YellowError
    global RedError

    img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    height = (data.height*2/3)
    mid = (data.width/2)
    img = img[height:, :]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img,BlueError = errorcal(img,hsv,mid,'Blue')
    img,GreenError = errorcal(img,hsv,mid,'Green')
    img,YellowError = errorcal(img,hsv,mid,'Yellow')
    img,RedError = errorcal(img,hsv,mid,'Red')
   
    # print("BlueError : ",BlueError)
    # print("GreenError : ",GreenError)
    # print("YellowError : ",YellowError)
    # print("RedError : ",RedError)
    # cv.imshow('image', cv.resize(img, (img.shape[1]/4, img.shape[0]/4), interpolation= cv.INTER_LINEAR))
    # cv.waitKey(1)

    imgpub.publish(bridge.cv2_to_imgmsg(cv.resize(img, (img.shape[1]/4, img.shape[0]/4), interpolation= cv.INTER_LINEAR)))

def errorcal(img,hsv,mid,color):
    mask = cv.inRange(hsv, np.array(colordict[color][0]), np.array(colordict[color][1]))
    M = cv.moments(mask)
    _,contours,hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    error = [0,0]
    for i, cnt in enumerate(contours):
        # print("contour : ",cv.contourArea(cnt))
        if cv.contourArea(cnt) < 40000:
            try:
                cX = int(M["m10"] / M["m00"])
                Xerror = mid-cX
                cY = int(M["m01"] / M["m00"])
                Yerror = img.shape[0]-70-cY
                # print("img.shape : ",img.shape)
                # print("cY : ",cY)
                # print("Yerror : ",Yerror)
                error = [Xerror,Yerror]
                cv.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv.putText(img, "centroid "+color, (cX - 50, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
            except ZeroDivisionError:
                pass
    return img,error

def heading(color):
    global BlueError
    global GreenError
    global YellowError
    global RedError
    
    while 1 :
        if color == 'Blue':
            Xerror = BlueError[0]
            Yerror = BlueError[1]
        if color == 'Red':
            Xerror = RedError[0]
            Yerror = RedError[1]
        if color == 'Green':
            Xerror = GreenError[0]
            Yerror = GreenError[1]
        if color == 'Yellow':
            Xerror = YellowError[0]
            Yerror = YellowError[1]
        # print("Xerror = ",Xerror)
        # print("Yerror = ",Yerror)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = Xerror*0.002
        pub.publish(twist)

        if 5>Xerror >-5:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)
            break
    
    while 1 :
        if color == 'Blue':
            Xerror = BlueError[0]
            Yerror = BlueError[1]
        if color == 'Red':
            Xerror = RedError[0]
            Yerror = RedError[1]
        if color == 'Green':
            Xerror = GreenError[0]
            Yerror = GreenError[1]
        if color == 'Yellow':
            Xerror = YellowError[0]
            Yerror = YellowError[1]
        # print("Xerror = ",Xerror)
        # print("Yerror = ",Yerror)
        twist = Twist()
        twist.linear.x = Yerror*0.0025; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        if 5>Yerror >-5:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)
            break

def rightspin():
    starttime = time.time()
    while time.time() - starttime < 6:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -1.0
        pub.publish(twist)

    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)

BlueError = [0,0]
GreenError = [0,0]
YellowError = [0,0]
RedError = [0,0]

if __name__ == '__main__':
    rospy.init_node('Game_control')
    sub = rospy.Subscriber('camera/rgb/image_raw', Image, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    imgpub = rospy.Publisher('imagebox', Image,queue_size=10)

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    controlArm('prepick')
    controlGripper('open')
    resp = move_base_client(*prepoint[0])
    resp = move_base_client(*waypoints[0])
    heading('Blue')
    controlArm('pick')
    controlGripper('close')
    controlArm('prepick')
    resp = move_base_client(*placepoint[0])
    controlArm('pick')
    controlGripper('open')
    controlArm('prepick')
    rightspin()
    
    resp = move_base_client(*waypoints[1])
    heading('Yellow')
    controlArm('pick')
    controlGripper('close')
    controlArm('prepick')
    resp = move_base_client(*placepoint[1])
    controlArm('pick')
    controlGripper('open')
    controlArm('prepick')
    rightspin()

    resp = move_base_client(*waypoints[2])
    heading('Red')
    controlArm('pick')
    controlGripper('close')
    controlArm('prepick')
    resp = move_base_client(*placepoint[2])
    controlArm('pick')
    controlGripper('open')
    controlArm('prepick')
    rightspin()

    resp = move_base_client(*waypoints[3])
    heading('Green')
    controlArm('pick')
    controlGripper('close')
    controlArm('prepick')
    resp = move_base_client(*placepoint[3])
    controlArm('pick')
    controlGripper('open')
    controlArm('prepick')
    rightspin()


    rospy.spin()
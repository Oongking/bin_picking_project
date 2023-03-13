#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

# Image
import cv2

# 3D
import open3d as o3d

# Utility
from utils import *
import getch
import numpy as np
import time
import copy
import sys
import geometry_msgs.msg



def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def resize(img,scale_percent):
	# scale_percent = 50 # percent of original size
	width = int(img.shape[1] * scale_percent / 100)
	height = int(img.shape[0] * scale_percent / 100)
	dim = (width, height)
	
			# resize image
	resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
	return resized

def control_plannar_arm(pose):

	waypoints = []
	pose_goal = move_group.get_current_pose().pose

	#### Single goal point
	pose_goal.position.x = pose[0]
	pose_goal.position.y = pose[1]
	pose_goal.position.z = pose[2]
	pose_goal.orientation.x = pose[3]
	pose_goal.orientation.y = pose[4]
	pose_goal.orientation.z = pose[5]
	pose_goal.orientation.w = pose[6]

	waypoints.append(copy.deepcopy(pose_goal))


	(plan, fraction) = move_group.compute_cartesian_path(
					waypoints, 0.001, 0.0  # waypoints to follow  # eef_step
			)  # jump_threshold

	# print(f"plan : {len(plan.joint_trajectory.points) }")

	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
			# Publish
	display_trajectory_publisher.publish(display_trajectory)

	move_group.execute(plan, wait=True)
	rate.sleep()



rospy.init_node('ur5posecheck', anonymous=True)
rate = rospy.Rate(10) # 10hz

# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

pose_goal = geometry_msgs.msg.Pose()

isGetPosition=False
isKeep=False

camera = 'zivid'

if camera == 'zivid':
	cam = zivid_cam()
	boardA3 = zivid_boardA3
	matrix_coefficients = zivid_matrix_coefficients
	distortion_coefficients = zivid_distortion_coefficients
	offset_x = A3_zivid_offset_x
	offset_y = A3_zivid_offset_y

save_pose = []
listener = tf.TransformListener()

buffer_path = "/home/oongking/RobotArm_ws/src/bin_picking/script/calibrate_buffer/"

while not rospy.is_shutdown():

	# print(move_group.get_current_pose().pose)

	key = getch.getch().lower()

	print("key : ",key)

	if key == 'j':
		joint_goal = move_group.get_current_joint_values()
		print(f" joint_goal : ",joint_goal)

	if key == 'w':
		rgb_image, depth_image = cam.get_rgbd()
		while True:
			cv2.imshow("Original Image",resize(rgb_image,50))
			if cv2.waitKey(1) & 0xFF==ord('q'):
				cv2.destroyAllWindows()
				break

	if key == 'p':
		print(f" Preview ")

		pose_now = move_group.get_current_pose().pose
		arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
																						quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
		print("arm_TFM : ",np.asarray(arm_TFM))

		Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.5,(0,0,0))

		Robotcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
		Robotcoor.transform(arm_TFM)

		ARCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
		ARoffset_transform = np.array([[1,0,0,0.2],
																		[0,1,0,0],
																		[0,0,1,0],
																		[0,0,0,1]])
		RotationAR = np.matmul( Ry(-90), Rx(90))
		ARoffset_transform[:3,:3] = RotationAR
		arpose = np.matmul(arm_TFM , ARoffset_transform)
		ARCoor.transform(arpose)

		o3d.visualization.draw_geometries([Realcoor,Robotcoor,ARCoor])

	if key == 's':
		pose_now = move_group.get_current_pose().pose
		posePrearray = np.array([   pose_now.position.x,pose_now.position.y,pose_now.position.z,
																pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w])
		save_pose.append(posePrearray)
		print(f" Num pose : ",len(save_pose))
		cv2.imwrite(buffer_path+f"pic/chessboard"+str(len(pose)).zfill(5)+".png",rgb_image)

	if key == 'z':
		for i,x in enumerate(save_pose):
			print(f"\n:: pose {i+1}: \n",x)
		robot_pose_path = buffer_path + "robot_chessboard_pos"
		np.save(robot_pose_path, save_pose)
		robot_pose = np.load(robot_pose_path+'.npy')
		print("robot_pose : ",robot_pose)
		break

	if key == 't':

		save_pose = []

		robot_pose_path =  buffer_path + "robot_chessboard_pos"
		robot_pose = np.load(robot_pose_path+'.npy')

		for i,pose in enumerate(robot_pose):
			print(f"pos : {pose}")
			control_plannar_arm(pose)
			rgb_image, depth_image = cam.get_rgbd()
			cv2.imwrite(buffer_path+f"pic/chessboard"+str(i).zfill(5)+".png",rgb_image)



		width=7
		height=6
		square_size = 0.030

		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		objp = np.zeros((height*width, 3), np.float32)
		objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

		objp = objp * square_size

		objpoints = []  # 3d point in real world space
		imgpoints = []  # 2d points in image plane.

		for i in range(len(robot_pose)):
			print(f"pic : {i}")
			img = cv2.imread(buffer_path+f"pic/chessboard"+str(i).zfill(5)+".png")
			
			# cv2.imshow('img',img)
			# cv2.waitKey(0)
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			
			# cv2.imshow('gray',gray)
			# cv2.waitKey(0)
			gray = cv2.bitwise_not(gray)
			# cv2.imshow('gray',gray)
			# cv2.waitKey(0)

			ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
			# print("corners : ",corners)
			if ret:
				objpoints.append(objp)

				corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
				imgpoints.append(corners2)

				# Draw and display the corners
				img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
				# cv2.imshow('img', img)
				# cv2.waitKey(500)

		cv2.imshow('img',img)
		cv2.waitKey(0)

		np.set_printoptions(suppress=True)
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

		f = open(buffer_path + "ZividCamMatrix.txt" ,"w+")
		f.write("mtx : ")
		f.write(str(mtx))
		f.write('\n')
		f.write("dist : ")
		f.write(str(dist))
		f.write('\n')

	rate.sleep()

# def checkUR5():
    
# if __name__ == '__main__':
#     try:
#         checkUR5()
#     except rospy.ROSInterruptException:
#         pass
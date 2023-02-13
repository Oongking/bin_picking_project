#!/usr/bin/env python

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

# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
# move_group.set_max_velocity_scaling_factor(0.1)
# move_group.set_max_acceleration_scaling_factor(0.1)
# move_group.set_goal_orientation_tolerance(np.deg2rad(5))
# move_group.set_goal_position_tolerance (0.01)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Constant Variable
D2R = np.pi/180
R2D = 180/np.pi

setupArBoardpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/SetUpPose.out',delimiter=',')
calibrationPose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/calibrationPose.out',delimiter=',')

# camera = 'azure'
camera = 'zivid'

rospy.init_node("RobotPoseCalibration", anonymous=True)

if camera == 'azure':
    cam = Azure_cam()
    boardA3 = azure_boardA3
    matrix_coefficients = azure_matrix_coefficients
    distortion_coefficients = azure_distortion_coefficients
    offset_x = A3_azure_offset_x
    offset_y = A3_azure_offset_y

if camera == 'zivid':
    cam = zivid_cam()
    boardA3 = zivid_boardA3
    matrix_coefficients = zivid_matrix_coefficients
    distortion_coefficients = zivid_distortion_coefficients
    offset_x = A3_zivid_offset_x
    offset_y = A3_zivid_offset_y

class RobotPoseCalibration:
    def __init__(self):
        
        rospy.loginfo(":: Starting RobotPose ::")

        self.Invert_arm_TFM = np.eye(4)
        self.ARoffset_transform = np.array([[1,0,0,0.27],
                                            [0,1,0,0],
                                            [0,0,1,0],
                                            [0,0,0,1]])

        self.Cam_AR_TFM = np.array([[1,0,0,0],
                                    [0,1,0,0],
                                    [0,0,1,0],
                                    [0,0,0,1]])

        RotationAR = np.matmul( Ry(-90), Rx(90))
        self.ARoffset_transform[:3,:3] = RotationAR
        self.Robot_TFM_list = []

    def add_AR_pose(self,Cam_AR_TFM):
        self.Cam_AR_TFM = Cam_AR_TFM
        Rpose = move_group.get_current_pose().pose
        arm_TFM = concatenate_matrices(translation_matrix((Rpose.position.x,Rpose.position.y,Rpose.position.z)), 
                                                    quaternion_matrix((Rpose.orientation.x,Rpose.orientation.y,Rpose.orientation.z,Rpose.orientation.w)))
        self.Invert_arm_TFM = inverse_matrix(arm_TFM)

        robotpose = np.matmul(Cam_AR_TFM , inverse_matrix(self.ARoffset_transform))
        robotpose = np.matmul(robotpose , self.Invert_arm_TFM)

        self.Robot_TFM_list.append(robotpose)

    def averageQuaternions(self,Q):
        # Number of quaternions to average
        M = Q.shape[0]
        A = np.zeros(shape=(4,4))

        for i in range(0,M):
            q = Q[i,:]
            # multiply q with its transposed version q' and add A
            A = numpy.outer(q,q) + A

        # scale
        A = (1.0/M)*A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = numpy.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = np.matrix(eigenVectors[:,eigenValues.argsort()[::-1]])
        # return the real part of the largest eigenvector (has only real part)
        return numpy.real(eigenVectors[:,0].getA1())

    def get_robot_pose(self):
        avg_robot_TFM = np.eye(4)
        translation = []
        Quaternions = []
        for TFM in self.Robot_TFM_list:
            translation.append(TFM[:3,3])
            Quaternions.append(quaternion_from_matrix(TFM))
        avgTranslation = np.asarray(np.median(translation, axis = 0))
        avg_robot_TFM[:3,3] = avgTranslation.transpose()

        q = self.averageQuaternions(np.asarray(Quaternions))
        
        avg_robot_TFM[:3,:3] = quaternion_matrix(q)[:3,:3]


        Robotcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        Robotcoor.transform(avg_robot_TFM)

        ARCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        ARCoor.transform(self.Cam_AR_TFM)

        # o3d.visualization.draw_geometries([Realcoor,Robotcoor,ARCoor])

        return avg_robot_TFM , Robotcoor,ARCoor

    


# by spinmyhead
# def setupArBoard(loop = False):
#     pose_goal = geometry_msgs.msg.Pose()

#     for i,pose in enumerate(setupArBoardpose):
#         pose_goal.position.x = pose[0]
#         pose_goal.position.y = pose[1]
#         pose_goal.position.z = pose[2]
#         pose_goal.orientation.x = pose[3]
#         pose_goal.orientation.y = pose[4]
#         pose_goal.orientation.z = pose[5]
#         pose_goal.orientation.w = pose[6]

#         move_group.set_pose_target(pose_goal)

#         plan = move_group.go(wait=True)
#         move_group.stop()
#         move_group.clear_pose_targets()

#         if i+1< len(setupArBoardpose) and not loop:
#             print(" Press G : Next ")
#             while 1:
                
#                 key = cv2.waitKey(1)

#                 if key & 0xFF==ord('g'):
#                     break

# by Cartesian
def setupArBoard(loop = False):
    pose_goal = geometry_msgs.msg.Pose()

    for i,pose in enumerate(setupArBoardpose):
        waypoints = []

        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
        # move_group.set_pose_target(pose_goal)
        waypoints.append(copy.deepcopy(pose_goal))

        (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.001, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
            # Publish
        display_trajectory_publisher.publish(display_trajectory)

        move_group.execute(plan, wait=True)
        # move_group.go(wait=True)

        if i+1< len(setupArBoardpose) and not loop:
            break

def CalibrateArBoard(loop = False):
    pose_goal = geometry_msgs.msg.Pose()

    for i,pose in enumerate(calibrationPose):
        waypoints = []

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

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
            # Publish
        display_trajectory_publisher.publish(display_trajectory)

        move_group.execute(plan, wait=True)


ur5 = RobotPoseCalibration()

br = tf.TransformBroadcaster()
listener = tf.TransformListener()

rgb_image = np.array([])
while not rospy.is_shutdown():

    print("=================================================================================")
    print("\n:: Key command ::\n\tc : Capturing & Estimate Pose\n\tp : Set PickAR Pose\n\tt : Loop Pose Calibrate\n\tq : Save Robot Pose\n\te : Shutdown the Server")
    key = getch.getch().lower()
    print("key : ",key)

    if key == 'a': # Preview Alu Process
        pcd, rgb_image, depth_image, pcd_env = cam.testMatrix()
        while True:
            cv2.imshow("Original Image",rgb_image)
            if cv2.waitKey(1) & 0xFF==ord('q'):
                cv2.destroyAllWindows()
                break
        # cv2.imwrite(f"/home/oongking/RobotArm_ws/src/bin_picking/script/data/calibration_alu.png",rgb_image)

        o3d.visualization.draw_geometries([pcd])

    if key == 'p':
        print(":: Attach ARBoard Pose ::")
        setupArBoard()
        print(":: Finish Attach ARBoard ::")
    
    if key == 't':
        print(":: test Calibration ARBoard Pose ::")
        CalibrateArBoard()
        print(":: Finish Calibration ARBoard ::")

    if key == 'q':
        print(ur5.get_robot_pose()[0])
        tf_toRobot,Robotcoor,ARCoor = ur5.get_robot_pose()

        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))

        if camera == 'azure':
            pcd = cam.buildPCD()
        elif camera == 'zivid':
            pcd = cam.get_pcd()

        down_pcd = pcd.voxel_down_sample(voxel_size=0.005)
        o3d.visualization.draw_geometries([down_pcd,Realcoor,Robotcoor,ARCoor])

        np.savetxt('/home/oongking/RobotArm_ws/src/bin_picking/script/realRobotpose.txt', tf_toRobot, delimiter=',',header='real_robotpose Pose')
        load = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/realRobotpose.txt',delimiter=',')
        print("load : ",load)
        
        break

    if key == 'c':
        
        if camera == 'azure':
            rgb_image, depth_image = cam.get_rgbd()
        elif camera == 'zivid':
            rgb_image, depth_image = cam.get_rgbd()

        rvec=None
        tvec=None

        (corners, ids,rejected)= cv2.aruco.detectMarkers(rgb_image,arucoDictA3,parameters= arucoParams)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(rgb_image,corners,ids)
            _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, boardA3, matrix_coefficients, distortion_coefficients,rvec,tvec)
        
        transformation_matrix = np.array([  [1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]],
                                            dtype=float)

        transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

        q= tf.transformations.quaternion_from_matrix(transformation_matrix)
        vec= [offset_x,offset_y,0,0]
        global_offset=quaternion_multiply(quaternion_multiply(q, vec),quaternion_conjugate(q))
        tvec[0]=tvec[0]+global_offset[0]
        tvec[1]=tvec[1]+global_offset[1]
        tvec[2]=tvec[2]+global_offset[2]
        transformation_matrix[ :3, 3] = np.asarray(tvec).transpose()

        if rvec is not None and tvec is not None:
            cv2.aruco.drawAxis( rgb_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.08 )

        ur5.add_AR_pose(transformation_matrix)
    
    if key == 'e':
        
        break

cv2.destroyAllWindows()
#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

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
import sys
from utils import *
import getch
import numpy as np
import time
import copy

# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# process_now = 'sim'
process_now = 'zivid'
# process_now = 'azure'

# Setup ROS Node
rospy.init_node("PickAndPlace", anonymous=True)
if process_now  !='sim':
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

rate = rospy.Rate(10) # 10hz

# Sim or Real
if process_now =='sim':
    control_speed = 1
else:
    control_speed = 0.5

# Constant Variable
D2R = np.pi/180
R2D = 180/np.pi
# ARM
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(control_speed)
move_group.set_max_acceleration_scaling_factor(control_speed)
move_group.set_goal_orientation_tolerance(np.deg2rad(5))
move_group.set_goal_position_tolerance (0.001)
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)
# move_group.allow_replanning(True)

display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

# GRIPPER
if process_now =='sim':
    group_name = "gripper"
    group_gripper = moveit_commander.MoveGroupCommander(group_name)
    group_gripper.set_max_velocity_scaling_factor(control_speed)
    group_gripper.set_max_acceleration_scaling_factor(control_speed)  
else:
    def genCommand(keyword):
        """Update the command according to the character entered by the user."""

        if keyword == 'activate':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

        if keyword == 'reset':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

        if keyword == 'close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

            command.rPR = 255

        if keyword == 'full_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

            command.rPR = 0

        if keyword == 'half_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

            command.rPR = 127

        if keyword == 'grip_close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 50 # Force

            command.rPR = 250


        #If the command entered is a int, assign this value to rPRA
        # try:
        #     command.rPR = int(keyword)
        #     if command.rPR > 255:
        #         command.rPR = 255
        #     if command.rPR < 0:
        #         command.rPR = 0
        # except ValueError:
        #     pass

            # Speed control
        # if keyword == 'f':
        #     command.rSP += 25
        #     if command.rSP > 255:
        #         command.rSP = 255
        # if keyword == 'l':
        #     command.rSP -= 25
        #     if command.rSP < 0:
        #         command.rSP = 0
            # Force control
        # if keyword == 'i':
        #     command.rFR += 25
        #     if command.rFR > 255:
        #         command.rFR = 255
        # if keyword == 'd':
        #     command.rFR -= 25
        #     if command.rFR < 0:
        #         command.rFR = 0

        return command

    # command = genCommand('activate', command)
    # pub.publish(command)
    # rospy.sleep(0.1)

# Constant Variable
D2R = np.pi/180
R2D = 180/np.pi

if process_now == 'sim':
    cam = sim_cam(get_depth = False)
elif process_now == 'zivid':
    cam = zivid_cam()
elif process_now == 'azure':
    cam = Azure_cam()

class Arm_model:
    def __init__(self):
        
        rospy.loginfo(":: Starting RobotPose ::")
        if process_now == 'sim':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/simRobotpose.txt',delimiter=',')
        elif process_now == 'zivid' or process_now == 'azure':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/realRobotpose.txt',delimiter=',')

        self.pcd_merged = o3d.geometry.PointCloud()

        self.Robot_cam = np.linalg.inv(np.asarray(self.Robotpose))
        VecCam_Robot = self.Robot_cam[:3,3]/np.linalg.norm(self.Robot_cam[:3, 3])
        self.VecRobot_Cam = VecCam_Robot*0.5

        self.gripper_offset = [0,0, 0.165]
        # self.gripper_offset = [0,0, 0]


        self.Robot2box = np.eye(4)
        self.Robot2box[0,3] = self.VecRobot_Cam[0]+self.gripper_offset[0]
        self.Robot2box[1,3] = self.VecRobot_Cam[1]+self.gripper_offset[1]+0.08
        self.Robot2box[2,3] = self.VecRobot_Cam[2]+self.gripper_offset[2]
        self.Robot2box[:3,:3] = np.matmul(Ry(90),Rx((0)))
        
        # remove self.gripper_offset to crop correct box
        cam_box = copy.deepcopy(self.Robot2box)
        cam_box[0,3] -= self.gripper_offset[0]
        cam_box[1,3] -= self.gripper_offset[1]
        cam_box[2,3] -= self.gripper_offset[2]

        self.cam_box_tf = np.matmul(self.Robotpose,cam_box)

        if process_now == 'sim':
            x_offset = 0.003
        else:
            x_offset = -0.03

        self.cam_box = fixbox(self.cam_box_tf[:3, :3],self.cam_box_tf[:3,3],x_offset)

        self.MotionPlan = {
            'prepick' : [0.4189, -1.7976, 2.1467, -1.9373, -1.5358, -0.3840],
            'home' : [0, 0, 0, 0, 0, 0],
            'up' : [0, -1.5707, 0, -1.5707, 0, 0],
            'full_open' : [0],
            'half_open' : [0.4],
            'close' : [0.8028],
            'grip_close' : [0.789465]
            # 'grip_close' : [0.8028]
            }
        
        self.place_order = 0

    def control_arm(self,position,rotation):

        orientation = tf.transformations.quaternion_from_matrix(rotation)
        # not pricision offset
        # self.gripper_offset = np.matmul([0.16,0,0],rotation[:3,:3])
        # print("self.gripper_offset : ",self.gripper_offset)
        waypoints = []
        pose_goal = move_group.get_current_pose().pose

        #### Single goal point
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]
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
        rate.sleep()
    
    def common_Gripper(self,pose):
        if process_now == 'sim':
            print(pose)
            joint_goal = group_gripper.get_current_joint_values()
            joint_goal[0] = self.MotionPlan[pose][0]

            group_gripper.go(joint_goal,wait=True)
            group_gripper.stop()
            rate.sleep()
        else:
            command = genCommand(pose)
            pub.publish(command)
            # print("complete publish gripper")
            rospy.sleep(0.2)

    def common_Arm(self,pose):
        print(pose)
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.MotionPlan[pose][0]
        joint_goal[1] = self.MotionPlan[pose][1]
        joint_goal[2] = self.MotionPlan[pose][2]
        joint_goal[3] = self.MotionPlan[pose][3]
        joint_goal[4] = self.MotionPlan[pose][4]
        joint_goal[5] = self.MotionPlan[pose][5]

        move_group.go(joint_goal,wait=True)
        move_group.stop()
        rate.sleep()
    
    def common_Arm_tip(self,pose):
        print(pose)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[5] = pose

        move_group.go(joint_goal,wait=True)
        move_group.stop()
        rospy.sleep(0.5)

    def modeling(self):
        rospy.loginfo(":: Modeling Process ::")
        
        model_tf = self.Robot2box
        print(f"model_tf[:3,:3] : {model_tf[:3,:3]}")
        self.control_arm(model_tf[:3,3] ,model_tf)

        joint_goal = move_group.get_current_joint_values()
        angle = -3.1415

        # self.common_Gripper('grip_close')
        for i in range(10):
        
            self.common_Arm_tip(angle+(3.1415*2*i)/9)

            pose_now = move_group.get_current_pose().pose
            arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
                                                    quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
            arm_TFM[0,3] += 0
            arm_TFM[1,3] += 0
            arm_TFM[2,3] += -self.gripper_offset[2]
            
            # pricision offset
            offset = np.eye(4)
            offset[0,3] = 0.012
            offset[1,3] = 0
            offset[2,3] = 0

            arm_TFM = np.matmul(arm_TFM,offset)

            model_cam = np.matmul(self.Robotpose,arm_TFM)
            # model_cam[0,3] += 0.012
            self.capturemodel(model_cam)

        return self.pcd_merged
        # self.common_Arm('prepick')
    
    def capturemodel(self,model_cam):
        print("saving")
        pcd = cam.buildPCD()

        pcd_crop = pcd.crop(self.cam_box)
        if not pcd_crop.is_empty():
            pcd_crop.translate(np.asarray(-model_cam[:3,3],dtype=np.float64),relative=True)
            pcd_crop.rotate(model_cam[:3, :3].transpose(),(0,0,0))
            pcd_crop.translate([0,0,0],relative=True)

            pcd_crop_filted, ind = pcd_crop.remove_statistical_outlier(nb_neighbors=100,std_ratio=0.5)
            self.pcd_merged += pcd_crop_filted
            # o3d.visualization.draw_geometries([self.pcd_merged])

            print(" :: Complete Capturing :: ")
        else:
            rospy.loginfo(" No Point Cloud In Workspace ")
    
    def get_robot_pose(self):
        return self.Robotpose


def fixbox(rot,trans,x_offset) :
    # Before rotate to canon
    y = 0.4
    x = 0.15
    z = 0.4
    
    fix_box = np.array([
    [x_offset,-y/2,z/2],
    [x_offset, y/2,z/2],
    [x_offset, y/2,-z/2],
    [x_offset,-y/2,-z/2],

    [x,-y/2,z/2],
    [x, y/2,z/2],
    [x,-y/2,-z/2],
    [x, y/2,-z/2]
    ])

    fixbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(fix_box))
    fixbox.rotate(rot,(0,0,0))
    fixbox.translate(np.asarray(trans,dtype=np.float64),relative=True)
    fixbox.color = (0, 0, 0)

    return fixbox 



if process_now == 'sim':
    cam = sim_cam(get_depth = False)
elif process_now == 'zivid':
    cam = zivid_cam()
elif process_now == 'azure':
    cam = Azure_cam()



capture_model = Arm_model()
model = o3d.geometry.PointCloud()

while not rospy.is_shutdown():

    print("=================================================================================")
    print("\n:: Key command ::\n\tc : Capturing & Preview Robotpose\n\tt : Full Process\n\tp : Preview Process\n\te : Shutdown the Server")
    key = getch.getch().lower() 
    print("key : ",key)

    if key == 'c':

        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        endeffCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        if process_now != 'zivid':
            pcd = cam.buildPCD()
        elif process_now == 'zivid':
            # rgb_image, depth_image, pcd = cam.capture()
            pcd = cam.buildPCD()

        RobotBaseCoor.transform(capture_model.get_robot_pose())
        endeffCoor.transform(capture_model.cam_box_tf)

        o3d.visualization.draw_geometries([endeffCoor,RobotBaseCoor,Realcoor,capture_model.cam_box])
        o3d.visualization.draw_geometries([pcd,endeffCoor,RobotBaseCoor,Realcoor,capture_model.cam_box])

    if key == 't':
        print(" Full Process ")
        model = capture_model.modeling()
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
        # center = sphere([0,0,0],0.01)
        o3d.visualization.draw_geometries([Realcoor,model])
    
    if key == 's':

        if not model.is_empty():
            print("Saving Pointcloud")
            o3d.io.write_point_cloud("/home/oongking/RobotArm_ws/src/bin_picking/script/data/model.pcd", model)
        else:
            print("Empty")

    if key == 'p':
        print(" Preview Process ")
        capture_model.control_arm(capture_model.Robot2box[:3,3],capture_model.Robot2box)


    if key == 'e':

        break

cv2.destroyAllWindows()
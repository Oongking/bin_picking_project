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

# cam_type = 'sim'
cam_type = 'zivid'
# cam_type = 'azure'

# Setup ROS Node
rospy.init_node("PickAndPlace", anonymous=True)
if cam_type  !='sim':
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
rate = rospy.Rate(10) # 10hz

# Real
# control_speed = 0.5
# Sim
control_speed = 1
# ARM
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# save when not use
move_group.set_max_velocity_scaling_factor(control_speed)
move_group.set_max_acceleration_scaling_factor(control_speed)

move_group.set_goal_orientation_tolerance(0.02)
move_group.set_goal_position_tolerance (0.001)
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)
move_group.allow_replanning(True)

display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

# GRIPPER
if cam_type =='sim':
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
            command.rFR  = 5 # Force

        if keyword == 'reset':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

        if keyword == 'close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 255

        if keyword == 'full_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 0

        if keyword == 'half_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 127

        if keyword == 'release_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 200

        if keyword == 'grip_close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

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

if cam_type == 'sim':
    cam = sim_cam(get_depth = False)
elif cam_type == 'zivid':
    cam = zivid_cam()
elif cam_type == 'azure':
    cam = Azure_cam()

class RobotPickPlace:
    def __init__(self):
        
        rospy.loginfo(":: Starting RobotPose ::")
        if cam_type == 'sim':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/simRobotpose.txt',delimiter=',')
        elif cam_type == 'zivid' or cam_type == 'azure':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/realRobotpose.txt',delimiter=',')

        offset_1 = np.eye(4,dtype=float)
        offset_1[:3, :3] = Rz(45)
        offset_1[:3,3] = np.asarray([0,0,0])

        offset_2 = np.eye(4,dtype=float)
        offset_2[:3,3] = np.asarray([0.425,-0.0275,0])
        
        offset_tf = np.matmul(offset_1,offset_2)
        
        cam_box_tf = np.matmul(self.Robotpose,offset_tf)

        if cam_type == 'sim':
            z_offset = 0.003
        else:
            z_offset = -0.01

        self.cam_box = fixbox(cam_box_tf[:3, :3],cam_box_tf[:3,3],z_offset)

        self.arm_box = fixbox(offset_tf[:3, :3],offset_tf[:3,3],z_offset)

        self.MotionPlan = {
            'prepick' : [0.4189, -1.7976, 2.1467, -1.9373, -1.5358, -0.3840],
            'home' : [0, 0, 0, 0, 0, 0],
            'up' : [0, -1.5707, 0, -1.5707, 0, 0],
            'full_open' : [0],
            'half_open' : [0.4],
            'release_open' : [0.7],
            'close' : [0.8028],
            'grip_close' : [0.789465]
            # 'grip_close' : [0.8028]
            }
        
        self.place_order = 0
    
    def set_cam_box(self,ws_tf):
        self.cam_box = fixbox(ws_tf[:3, :3],ws_tf[:3,3],0, x = 0.3, y = 0.3, z = 0.1)

    def reload(self):
        if cam_type == 'sim':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/simRobotpose.txt',delimiter=',')
        elif cam_type == 'zivid' or cam_type == 'azure':
            self.Robotpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/realRobotpose.txt',delimiter=',')

    def get_robot_pose(self):
        return self.Robotpose

    def choose_pose(self,pcd):
        down_pcd = pcd.voxel_down_sample(voxel_size=0.005)
        down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=50))
        volume,pcds = Cluster(down_pcd, Clus_eps=0.02, point_upper_limit = 200, point_lower_limit = 20)

        center = []
        angle = []
        pcd_sphere = []
        pcd_cylinders = []
        for pcd in pcds:
            center.append(pcd.get_center())
            cen = sphere(pcd.get_center(),radius= 0.005)
            pcd_sphere.append(cen.pcd)

            mean, covar = pcd.compute_mean_and_covariance()
            w,v=np.linalg.eig(covar)
            v = v.transpose()
            w, v = zip(*sorted(zip(w, v),reverse=True))

            mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.002,
                                                            height=0.03)
            mesh_cylinder.compute_vertex_normals()
            mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
            mesh_cylinder.rotate(rotation_matrix_from_vectors([0,0,1], v[0])[0],(0,0,0))
            mesh_cylinder.translate(np.asarray(mean,dtype=np.float64),relative=False)
            pcd_cylinders.append(mesh_cylinder)
            vector = np.asarray(v[0])
            if vector[1]>0:
                vector *= -1
            angle.append(rotation_matrix_from_vectors([1,0,0], vector)[1])

        # o3d.visualization.draw_geometries([Realcoor]+pcds+pcd_sphere+pcd_cylinders)

        return pcd_sphere,pcd_cylinders,center,angle
    
    def pick_single_process(self,centers,angles):
        rospy.loginfo(":: Pick Process ::")
        self.common_Gripper('full_open')
        self.common_Arm('prepick')

        for i,(center,angle) in enumerate(zip(centers,angles)):
            # if i == 1:
            #     break
            # print("angle*R2D : ",angle*R2D)
            pre_center = copy.deepcopy(center)
            pre_center[2] += 0.05
            # print(f"center : {center} , pre_center : {pre_center}")

            rotation = np.eye(4)
            if (angle*R2D)> 90:
                angleD = (angle*R2D)-180
            else:
                angleD = (angle*R2D)
            rotation[:3,:3] = np.matmul(Ry(90),Rx((angleD)))
            
            # rotation[:3,:3] = np.matmul(Ry(90),Rx((angle*R2D)))

            self.control_arm(pre_center,rotation)
            self.control_arm(center,rotation)
            self.common_Gripper('grip_close')
            self.control_arm(pre_center,rotation)
            rospy.sleep(1.)

            # place
            # self.control_arm((0.4,0.1,0.35),rotation)
            # self.common_Gripper('half_open')
            self.control_arm([0.3,0,0.15],rotation)
            self.place_inorder()

        self.common_Arm('prepick')

    def pick_multi_process(self,center,angle):
        rospy.loginfo(":: Pick Process ::")
        self.common_Gripper('full_open')
        self.common_Arm('prepick')


        pre_center = copy.deepcopy(center)
        pre_center[2] += 0.05
        # print(f"center : {center} , pre_center : {pre_center}")

        rotation = np.eye(4)
        if (angle*R2D)> 90:
            angleD = (angle*R2D)-180
        else:
            angleD = (angle*R2D)
        
        print(f"angleD : {angleD}")
        rotation[:3,:3] = np.matmul(Ry(90),Rx((angleD)))
        
        # rotation[:3,:3] = np.matmul(Ry(90),Rx((angle*R2D)))

        self.control_arm(pre_center,rotation)
        self.control_arm(center,rotation)
        self.common_Gripper('grip_close')
        self.control_arm(pre_center,rotation)
        rospy.sleep(1.)

        # place
        # self.control_arm((0.4,0.1,0.35),rotation)
        # self.common_Gripper('half_open')
        self.control_arm([0.3,0,0.15],rotation)
        self.place_inorder()


    def place_inorder(self):

        start_prepos = np.array([0.161,-0.161,0.1])
        start_pos = np.array([0.161,-0.161,0.01])
        rotation = np.eye(4)
        rotation[:3,:3] = np.matmul(Ry(90),Rx(45))

        order_offset = np.array([0.0254911995,0.0254911995,0])
        pre_placepos = start_prepos+(order_offset*self.place_order)
        placepos = start_pos+(order_offset*self.place_order)

        self.control_arm(pre_placepos,rotation)
        rospy.sleep(1.)
        self.control_arm(placepos,rotation)

        #for  experiment
        print("====")
        print("\n:: Key command ::\n\tg : Go")
        # key = getch.getch().lower()
        # print("key : ",key)


        self.common_Gripper('release_open')
        rospy.sleep(1.)
        self.control_arm(pre_placepos,rotation)
        self.place_order +=1
        # rospy.loginfo(f":: Place_Order {self.place_order}:: \n pre_placepos : {pre_placepos}\n placepos : {placepos}")


    def control_arm(self,position,rotation):

        orientation = tf.transformations.quaternion_from_matrix(rotation)
        # not pricision offset
        # gripper_offset = np.matmul([0.16,0,0],rotation[:3,:3])
        # print("gripper_offset : ",gripper_offset)
        gripper_offset = [0,0, 0.165]
        waypoints = []
        pose_goal = move_group.get_current_pose().pose

        #### Single goal point
        
        pose_goal.position.x = position[0]+gripper_offset[0]
        pose_goal.position.y = position[1]+gripper_offset[1]
        pose_goal.position.z = position[2]+gripper_offset[2]
        # print(f"pose_goal.position.z : {pose_goal.position.z}")
        if pose_goal.position.z < 0.17:
            pose_goal.position.z = 0.17
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]
        # move_group.set_pose_target(pose_goal)
        
        waypoints.append(copy.deepcopy(pose_goal))
        

        ### Trajectory goal point
        # position[0] = position[0]+gripper_offset[0]
        # position[1] = position[1]+gripper_offset[1]
        # position[2] = position[2]+gripper_offset[2]

        # startposition = np.array([pose_goal.position.x,pose_goal.position.y,pose_goal.position.z])
        # goalposition = np.array([position[0],position[1],position[2]])
        # position_vector = goalposition-startposition

        # for i in range(1,11):

        #     pose_goal.position.x = startposition[0]+(position_vector[0]*(i/10))
        #     pose_goal.position.y = startposition[1]+(position_vector[1]*(i/10))
        #     pose_goal.position.z = startposition[2]+(position_vector[2]*(i/10))
        #     pose_goal.orientation.x = orientation[0]
        #     pose_goal.orientation.y = orientation[1]
        #     pose_goal.orientation.z = orientation[2]
        #     pose_goal.orientation.w = orientation[3]
        #     move_group.set_pose_target(pose_goal)
            
        #     waypoints.append(copy.deepcopy(pose_goal))
        ###



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

    def common_Gripper(self,pose):
        if cam_type == 'sim':
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

# True w8 for pcd
# pcd2 = cam.get_pcd()
# pcd2.paint_uniform_color([1, 0.706, 0])
pickplace = RobotPickPlace()

pcd_model_eraser = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/eraser/eraser.pcd")
pcd_model_shampoo = o3d.io.read_point_cloud("/home/oongking/RobotArm_ws/src/model3d/script/buildModel/Data/shampoo/shampoo.pcd")

while not rospy.is_shutdown():

    print("=================================================================================")
    print("\n:: Preview Key command ::\n\ta : Preview ArUco board\n\tc : Preview Robotpose\n\tu : Preview Alu Process"+
            "\n\n:: Setup Config Key command ::\n\ts : Set Cam Workspace"+
            "\n\n:: Alu Process Key command ::\n\tt : Full Single Alu Capture Process\n\ty : Full Multi Alu Capture Process")
    key = getch.getch().lower()
    print("key : ",key)



    ### =========== Preview =========== ###

    if key == 'a': # Preview Alu Process
        rgb_image, depth_image, pcd_env = cam.capture()
        Ar_tfm = workspace_ar_set(rgb_image, camera = cam_type, show = True)
        ws_box = fixbox(Ar_tfm[:3, :3],Ar_tfm[:3,3],0, x = 0.3, y = 0.3, z = 0.1)
        ws_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
        ws_coor.rotate(Ar_tfm[:3, :3],(0,0,0))
        ws_coor.translate(np.asarray(Ar_tfm[:3,3],dtype=np.float64),relative=True)
        o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box])

    if key == 'c': # Preview Robotpose
        pickplace.reload()
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        if cam_type != 'zivid':
            pcd = cam.buildPCD()
        elif cam_type == 'zivid':
            rgb_image, depth_image, pcd = cam.capture()

        RobotBaseCoor.transform(pickplace.get_robot_pose())

        o3d.visualization.draw_geometries([RobotBaseCoor,Realcoor,pickplace.cam_box])
        o3d.visualization.draw_geometries([pcd,RobotBaseCoor,Realcoor,pickplace.cam_box])

    if key == 'u': # Preview Alu Process
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        # if cam_type != 'zivid':
        pcd = cam.buildPCD()
        # elif cam_type == 'zivid':
        #     rgb_image, pcd = cam.capture()

        RobotBaseCoor.transform(pickplace.get_robot_pose())

        o3d.visualization.draw_geometries([pcd,RobotBaseCoor,Realcoor,pickplace.cam_box])

        inv_tf = np.linalg.inv(pickplace.get_robot_pose())
        pcd.transform(inv_tf)

        crop_pcd = pcd.crop(pickplace.arm_box)
        o3d.visualization.draw_geometries([crop_pcd,Realcoor,pickplace.arm_box])

        pcd_sphere,pcd_cylinders,center,angle = pickplace.choose_pose(crop_pcd)

        o3d.visualization.draw_geometries([Realcoor,pickplace.arm_box]+pcd_sphere+pcd_cylinders)
        o3d.visualization.draw_geometries([pcd,Realcoor,pickplace.arm_box]+pcd_sphere+pcd_cylinders)



    ### =========== Setup Config =========== ###

    if key == 's': # Set Cam Workspace
        rgb_image, depth_image = cam.get_rgbd()
        Ar_tfm = workspace_ar_set(rgb_image, camera = cam_type, show = False)
        pickplace.set_cam_box(Ar_tfm)



    ### =========== Alu Process =========== ###

    if key == 't': # Full Single Alu Capture Process
        pickplace.place_order = 0
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor.transform(pickplace.get_robot_pose())
        inv_tf = np.linalg.inv(pickplace.get_robot_pose())
        
        pcd = cam.buildPCD()

        # o3d.visualization.draw_geometries([pcd,RobotBaseCoor,Realcoor,pickplace.cam_box])

        pcd.transform(inv_tf)

        crop_pcd = pcd.crop(pickplace.arm_box)

        if np.asarray(crop_pcd.points).shape[0]>200:
            # o3d.visualization.draw_geometries([crop_pcd,Realcoor,pickplace.arm_box])
            pcd_sphere,pcd_cylinders,center,angle = pickplace.choose_pose(crop_pcd)

            o3d.visualization.draw_geometries([pcd,Realcoor,pickplace.arm_box]+pcd_sphere+pcd_cylinders)

            pickplace.pick_single_process(center,angle)
        else:
            print(" No Object Detected")

    if key == 'y': # Full Multi Alu Capture Process
        pickplace.place_order = 0

        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor.transform(pickplace.get_robot_pose())
        inv_tf = np.linalg.inv(pickplace.get_robot_pose())

        while True:

            pcd = cam.buildPCD()

            # o3d.visualization.draw_geometries([pcd,RobotBaseCoor,Realcoor,pickplace.cam_box])
            
            pcd.transform(inv_tf)

            crop_pcd = pcd.crop(pickplace.arm_box)
            print(np.asarray(crop_pcd.points).shape[0])
            # o3d.visualization.draw_geometries([crop_pcd,Realcoor])
            if np.asarray(crop_pcd.points).shape[0]>200:
                # o3d.visualization.draw_geometries([crop_pcd,Realcoor,pickplace.arm_box])
                pcd_sphere,pcd_cylinders,center,angle = pickplace.choose_pose(crop_pcd)
                # o3d.visualization.draw_geometries([pcd,Realcoor,pickplace.arm_box]+pcd_sphere+pcd_cylinders)

                pickplace.pick_multi_process(center[0],angle[0])

            else:

                pickplace.common_Arm('prepick')
                break

   

    ### =========== Shampoo Process =========== ###

    if key == 'h': # Full Multi Shampoo Capture Process
        pickplace.place_order = 0
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        RobotBaseCoor.transform(pickplace.get_robot_pose())
        inv_tf = np.linalg.inv(pickplace.get_robot_pose())

        while True:

            pcd = cam.buildPCD()
            crop_pcd = pcd.crop(pickplace.cam_box)
            crop_pcd.transform(inv_tf)
            print(np.asarray(crop_pcd.points).shape[0])

            # o3d.visualization.draw_geometries([crop_pcd,Realcoor])
            if np.asarray(crop_pcd.points).shape[0]>200:
                
                group_model = crop_pcd.voxel_down_sample(voxel_size=0.001)
                group_model, ind = group_model.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    
                volume,pcds = Cluster(group_model)

                obj_tf,fitnesses = obj_pose_estimate(pcd_model_shampoo,pcds,point_p_obj = 4000)
                obj_coor = coordinates(obj_tf)

                o3d.visualization.draw_geometries([ws_coor,Realcoor,pcd_env,ws_box,pcd_model_shampoo]+obj_coor)

                ############ multi capture process ############

            else:
                pickplace.common_Arm('prepick')
                break


            

    if key == 'e':

        break

cv2.destroyAllWindows()
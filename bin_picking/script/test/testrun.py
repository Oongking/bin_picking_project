#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import getch
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


control_speed = 1
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(control_speed)
move_group.set_max_acceleration_scaling_factor(control_speed)
move_group.set_goal_orientation_tolerance(np.deg2rad(5))
move_group.set_goal_position_tolerance (0.001)
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)

pose_goal = geometry_msgs.msg.Pose()

isGetPosition=False
isKeep=False

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

setupArBoardpose = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/poseTXT/SetUpPose.out',delimiter=',')

def setupArBoard(loop = False):
    pose_goal = move_group.get_current_pose().pose

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
        
        # print("Plan lenght : ",len(plan.joint_trajectory.points))

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
            # Publish
        display_trajectory_publisher.publish(display_trajectory)

        move_group.execute(plan, wait=True)
        # move_group.go(wait=True)


def talker():
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        

        waypoints = []
        

        key = getch.getch().lower()
        print("key : ",key)
        if key == 'g':
            

            wpose = move_group.get_current_pose().pose
            # wpose.position.x = 0.3249209854074963
            # wpose.position.y = 0.5654487304078913
            # wpose.position.z = 0.7543323330746082
            # wpose.orientation.x = 0.1484020002313711
            # wpose.orientation.y = 0.3458986611726757
            # wpose.orientation.z = 0.8481155815828019
            # wpose.orientation.w = 0.37286850604837135

            # waypoints.append(copy.deepcopy(wpose))

            wpose.position.x = 4.172217899999999813e-01
            wpose.position.y = 4.389113800000000176e-01
            wpose.position.z = 3.907582900000000081e-01
            wpose.orientation.x = -6.608203799999999850e-01
            wpose.orientation.y = 2.394723199999999885e-01
            wpose.orientation.z = 6.465039599999999886e-01
            wpose.orientation.w = 2.966514199999999990e-01
            move_group.set_pose_target(wpose)
            waypoints.append(copy.deepcopy(wpose))
            # waypoints.append(move_group.get_current_pose().pose)
            
            print("========================================================================")
            print("waypoints : ",waypoints)
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            print("plan :",plan)
            print("========================================================================")
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
                # Publish
            display_trajectory_publisher.publish(display_trajectory)

            move_group.execute(plan, wait=True)

        if key == 'h':
            waypoints = []

            wpose = move_group.get_current_pose().pose
            wpose.position.x = 0.3249209854074963
            wpose.position.y = 0.5654487304078913
            wpose.position.z = 0.7543323330746082
            wpose.orientation.x = 0.1484020002313711
            wpose.orientation.y = 0.3458986611726757
            wpose.orientation.z = 0.8481155815828019
            wpose.orientation.w = 0.37286850604837135
            move_group.set_pose_target(wpose)
            waypoints.append(copy.deepcopy(wpose))

            # wpose.position.x = 0.5714843337759893
            # wpose.position.y = 0.19062133908843937
            # wpose.position.z = 0.7994689510460969
            # wpose.orientation.x = 0.2443928335108608
            # wpose.orientation.y = 0.2440853775710941
            # wpose.orientation.z = 0.663041658503706
            # wpose.orientation.w = 0.6641311846865527

            # waypoints.append(copy.deepcopy(wpose))
            # waypoints.append(move_group.get_current_pose().pose)
            
            print("========================================================================")
            print("waypoints : ",waypoints)
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            print("plan :",plan)
            print("========================================================================")
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
                # Publish
            display_trajectory_publisher.publish(display_trajectory)

            move_group.execute(plan, wait=True)

        if key == 't':

            wpose = move_group.get_current_pose().pose
            # wpose.position.x = 0.3249209854074963
            # wpose.position.y = 0.5654487304078913
            # wpose.position.z = 0.7543323330746082
            # wpose.orientation.x = 0.1484020002313711
            # wpose.orientation.y = 0.3458986611726757
            # wpose.orientation.z = 0.8481155815828019
            # wpose.orientation.w = 0.37286850604837135

            wpose.position.x = 0.035489553156615505
            wpose.position.y = 0.6510428861434645
            wpose.position.z = 0.7544804905758455
            wpose.orientation.x = 0.0643055445213302
            wpose.orientation.y = 0.370669463483489
            wpose.orientation.z = 0.9114520412514046
            wpose.orientation.w = 0.16650562237752076

            move_group.set_pose_target(wpose)

            plan = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            # display_trajectory.trajectory_start = robot.get_current_state()
            # display_trajectory.trajectory.append(plan)
                # Publish
            # display_trajectory_publisher.publish(display_trajectory)
        
        if key == 'y':

            wpose = move_group.get_current_pose().pose
            wpose.position.x = 0.3249209854074963
            wpose.position.y = 0.5654487304078913
            wpose.position.z = 0.7543323330746082
            wpose.orientation.x = 0.1484020002313711
            wpose.orientation.y = 0.3458986611726757
            wpose.orientation.z = 0.8481155815828019
            wpose.orientation.w = 0.37286850604837135

            # wpose.position.x = 0.5714843337759893
            # wpose.position.y = 0.19062133908843937
            # wpose.position.z = 0.7994689510460969
            # wpose.orientation.x = 0.2443928335108608
            # wpose.orientation.y = 0.2440853775710941
            # wpose.orientation.z = 0.663041658503706
            # wpose.orientation.w = 0.6641311846865527

            move_group.set_pose_target(wpose)

            plan = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            # display_trajectory.trajectory_start = robot.get_current_state()
            # display_trajectory.trajectory.append(plan)
                # Publish
            # display_trajectory_publisher.publish(display_trajectory)

        if key == 'l':
            round = 1
            while 1:
                setupArBoard(loop = True)
                print("ROUND : ",round)
                round +=1
                rate.sleep()
                
                

        if key == 'q':
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
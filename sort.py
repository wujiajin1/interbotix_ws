#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
from object_color_detector.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
def sorting():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('sort', anonymous=True)

        scene = PlanningSceneInterface()

        arm = moveit_commander.MoveGroupCommander("interbotix_arm")
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        end_effector_link = arm.get_end_effector_link()

        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
                
        arm.allow_replanning(True)

        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)

        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)

        start_pose = arm.get_current_pose(end_effector_link)

        start_pose = PoseStamped()
        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.Time.now()     
        start_pose.pose.position.x = 0.195017173013889
        start_pose.pose.position.y = 0.009030844915655669
        start_pose.pose.position.z = 0.27941254275752253
        start_pose.pose.orientation.x = -0.010670034155991366
        start_pose.pose.orientation.y = 0.7324379348399804
        start_pose.pose.orientation.z = 0.026298509384963323
        start_pose.pose.orientation.w = 0.6802420233876937


        arm.set_pose_target(start_pose, end_effector_link)
        plan_success, traj, planning_time, error_code = arm.plan()
        print(plan_success)
        arm.execute(traj)
        rospy.sleep(1)
        arm.set_start_state_to_current_state()

        joint_positions = [0.036, -0.036]
        gripper.set_joint_value_target(joint_positions)   
        gripper.go()
        rospy.sleep(1)

        rospy.wait_for_service('/object_detect')

        # scene.remove_world_object('table')
        scene.remove_world_object('box1')
        scene.remove_world_object('box2')
        scene.remove_world_object('box3')
        box_size = [0.01, 0.01, 0.01]
        table_size = [0.6, 0.6, 0.02]


        while True:
            do_sort = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
            RedList = do_sort(0).redObjList
            GreenList = do_sort(1).greenObjList
            BlueList = do_sort(2).blueObjList
            if (RedList == [] and GreenList == [] and BlueList == []):break
            for i in RedList:
                print('red')
                x = i.position.y*-0.00038513609804759853 + 0.3272962092062134
                y = i.position.x*-0.000373989499124927 + 0.12331881823485288
                target_pose = PoseStamped()
                target_pose.header.frame_id = reference_frame
                target_pose.header.stamp = rospy.Time.now()     
                target_pose.pose.position.x = x + 0.02
                target_pose.pose.position.y = y - 0.01
                target_pose.pose.position.z = 0.09
                target_pose.pose.orientation.y = 0.7346795570804928
                target_pose.pose.orientation.w = 0.6784077011087422

                box_pose = deepcopy(target_pose)
                box_pose.pose.position.z = 0.01
                box_pose.pose.orientation.x = 0
                box_pose.pose.orientation.y = 0
                box_pose.pose.orientation.z = 0
                box_pose.pose.orientation.w = 1
                scene.add_box('box1', box_pose, box_size)

                arm.set_pose_target(target_pose, end_effector_link)
                # print(target_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

                joint_positions = [0.022, -0.022]
                gripper.set_joint_value_target(joint_positions)   
                gripper.go()
                rospy.sleep(1)
                scene.remove_world_object('box1')
                red_pose = PoseStamped()
                red_pose.header.frame_id = reference_frame
                red_pose.header.stamp = rospy.Time.now()     
                red_pose.pose.position.x = -0.00179
                red_pose.pose.position.y = 0.1977
                red_pose.pose.position.z = 0.25026
                red_pose.pose.orientation.x = -0.01835588661084443
                red_pose.pose.orientation.y = 0.7294872376086887 
                red_pose.pose.orientation.z = 0.012975931110630778
                red_pose.pose.orientation.w = 0.6836249386941544

                arm.set_pose_target(red_pose, end_effector_link)
                # print(red_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

                joint_positions = [0.036, -0.036]
                gripper.set_joint_value_target(joint_positions)   
                gripper.go()
                rospy.sleep(1)

                start_pose = PoseStamped()
                start_pose.header.frame_id = reference_frame
                start_pose.header.stamp = rospy.Time.now()     
                start_pose.pose.position.x = 0.195017173013889
                start_pose.pose.position.y = 0.009030844915655669
                start_pose.pose.position.z = 0.27941254275752253
                start_pose.pose.orientation.x = -0.010670034155991366
                start_pose.pose.orientation.y = 0.7324379348399804
                start_pose.pose.orientation.z = 0.026298509384963323
                start_pose.pose.orientation.w = 0.6802420233876937


                arm.set_pose_target(start_pose, end_effector_link)
                # print(start_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

            for i in BlueList:
                print('blue')
                x = i.position.y*-0.00038513609804759853 + 0.3272962092062134
                y = i.position.x*-0.000373989499124927 + 0.12331881823485288
                target_pose = PoseStamped()
                target_pose.header.frame_id = reference_frame
                target_pose.header.stamp = rospy.Time.now()     
                target_pose.pose.position.x = x + 0.01
                target_pose.pose.position.y = y -0.01
                target_pose.pose.position.z = 0.09
                target_pose.pose.orientation.y = 0.7346795570804928
                target_pose.pose.orientation.w = 0.6784077011087422
                # scene.add_box('box', target_pose, box_size)


                box_pose = deepcopy(target_pose)
                box_pose.pose.position.z = 0
                box_pose.pose.orientation.x = 0
                box_pose.pose.orientation.y = 0
                box_pose.pose.orientation.z = 0
                box_pose.pose.orientation.w = 1
                scene.add_box('box2', box_pose, box_size)

                arm.set_pose_target(target_pose, end_effector_link)
                # print(target_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

                joint_positions = [0.022, -0.022]
                gripper.set_joint_value_target(joint_positions)
                        
                gripper.go()
                rospy.sleep(1)
                
                blue_pose = PoseStamped()
                blue_pose.header.frame_id = reference_frame
                blue_pose.header.stamp = rospy.Time.now()     
                blue_pose.pose.position.x = -0.08489576271267496
                blue_pose.pose.position.y = 0.17379461737900895
                blue_pose.pose.position.z = 0.29694622336800486
                blue_pose.pose.orientation.x = 0.04454801659774753
                blue_pose.pose.orientation.y = 0.7210636852429514
                blue_pose.pose.orientation.z = 0.02648528977223971
                blue_pose.pose.orientation.w = 0.6909277570533269

                arm.set_pose_target(blue_pose, end_effector_link)
                # print(blue_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)
                scene.remove_world_object('box2')

                joint_positions = [0.036, -0.036]
                gripper.set_joint_value_target(joint_positions)   
                gripper.go()
                rospy.sleep(1)
                
                start_pose = PoseStamped()
                start_pose.header.frame_id = reference_frame
                start_pose.header.stamp = rospy.Time.now()     
                start_pose.pose.position.x = 0.195017173013889
                start_pose.pose.position.y = 0.009030844915655669
                start_pose.pose.position.z = 0.27941254275752253
                start_pose.pose.orientation.x = -0.010670034155991366
                start_pose.pose.orientation.y = 0.7324379348399804
                start_pose.pose.orientation.z = 0.026298509384963323
                start_pose.pose.orientation.w = 0.6802420233876937


                arm.set_pose_target(start_pose, end_effector_link)
                # print(start_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

            for i in GreenList:
                print('green')
                x = i.position.y*-0.00038513609804759853 + 0.3272962092062134
                y = i.position.x*-0.000373989499124927 + 0.12331881823485288
                target_pose = PoseStamped()
                target_pose.header.frame_id = reference_frame
                target_pose.header.stamp = rospy.Time.now()     
                target_pose.pose.position.x = x + 0.01
                target_pose.pose.position.y = y - 0.01
                target_pose.pose.position.z = 0.09
                target_pose.pose.orientation.y = 0.7346795570804928
                target_pose.pose.orientation.w = 0.6784077011087422
                # scene.add_box('box', target_pose, box_size)


                box_pose = deepcopy(target_pose)
                box_pose.pose.position.z = 0
                box_pose.pose.orientation.x = 0
                box_pose.pose.orientation.y = 0
                box_pose.pose.orientation.z = 0
                box_pose.pose.orientation.w = 1
                scene.add_box('box3', box_pose, box_size)

                arm.set_pose_target(target_pose, end_effector_link)
                # print(target_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

                joint_positions = [0.022, -0.022]
                gripper.set_joint_value_target(joint_positions)
                        
                gripper.go()
                rospy.sleep(1)
                
                green_pose = PoseStamped()
                green_pose.header.frame_id = reference_frame
                green_pose.header.stamp = rospy.Time.now()     
                green_pose.pose.position.x = 0.08272624825021539
                green_pose.pose.position.y = 0.17625306081178943
                green_pose.pose.position.z = 0.2724676982421477
                green_pose.pose.orientation.x = -0.03672471911590043
                green_pose.pose.orientation.y = 0.7017943034628552
                green_pose.pose.orientation.z = 0.028382943475927385
                green_pose.pose.orientation.w = 0.7108659924012307

                arm.set_pose_target(green_pose, end_effector_link)
                # print(green_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)
                scene.remove_world_object('box3')

                joint_positions = [0.036, -0.036]
                gripper.set_joint_value_target(joint_positions)   
                gripper.go()
                rospy.sleep(1)

                start_pose = PoseStamped()
                start_pose.header.frame_id = reference_frame
                start_pose.header.stamp = rospy.Time.now()     
                start_pose.pose.position.x = 0.195017173013889
                start_pose.pose.position.y = 0.009030844915655669
                start_pose.pose.position.z = 0.27941254275752253
                start_pose.pose.orientation.x = -0.010670034155991366
                start_pose.pose.orientation.y = 0.7324379348399804
                start_pose.pose.orientation.z = 0.026298509384963323
                start_pose.pose.orientation.w = 0.6802420233876937


                arm.set_pose_target(start_pose, end_effector_link)
                # print(start_pose)
                plan_success, traj, planning_time, error_code = arm.plan()
                print(plan_success)
                arm.execute(traj)
                rospy.sleep(1)

        arm.set_named_target('Sleep')
        arm.go()
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    except rospy.ServiceException:
        print ("Service listen fialed")


if __name__ == '__main__':
    sorting()
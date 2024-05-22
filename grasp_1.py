#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import numpy as np
import math
import yaml

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from object_color_detector.srv import *
from sklearn.linear_model import LinearRegression
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        def moveTo(x, y, z): 
    # Create pose data           
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            capture_quaternion = Quaternion(0.00, 2**.5/2., 0.00, 2**.5/2.)
            # capture_quaternion = Quaternion(0,1,0,0)
            target_pose.pose.orientation = capture_quaternion
    
    # Set pick position 
            arm.set_start_state_to_current_state()
            end_effector_link = arm.get_end_effector_link()
            arm.set_pose_target(target_pose, end_effector_link)
    
            arm.go()
            rospy.sleep(2)            
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        config_filename = '/home/ubuntu/interbotix_ws/src/object_color_detector/config/vision_config.yaml'

# 标定参数
        regObjList  = []
        # 初始化ROS节点
        rospy.init_node('grasp_mine', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        scene.remove_world_object('box1')
        scene.remove_world_object('box2')
        scene.remove_world_object('box3')
        box_size = [0.1, 0.1, 0.13]
        box1 = PoseStamped()
        box1.header.frame_id = 'world'
        box1.pose.position.x = 0.085
        box1.pose.position.y = 0.19
        box1.pose.position.z = 0.065
        box1.pose.orientation.w = 1.0
        scene.add_box('box1', box1, box_size)

        box2 = PoseStamped()
        box2.header.frame_id = 'world'
        box2.pose.position.x = -0.005
        box2.pose.position.y = 0.19
        box2.pose.position.z = 0.065
        box2.pose.orientation.w = 1.0
        scene.add_box('box2', box2, box_size)  

        box3 = PoseStamped()
        box3.header.frame_id = 'world'
        box3.pose.position.x = -0.090
        box3.pose.position.y = 0.19
        box3.pose.position.z = 0.065
        box3.pose.orientation.w = 1.0
        scene.add_box('box3', box3, box_size)

        arm = moveit_commander.MoveGroupCommander("interbotix_arm")
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.2)
        arm.set_max_velocity_scaling_factor(0.2)
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)

        moveTo(0.22, 0, 0.08)
        gripper.set_named_target('Closed')
        gripper.go()
        rospy.sleep(1)
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        moveTo(0.22, 0, 0.08)
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)
        # input_value = input("请将标定物放置到夹爪正下方，按任意键确认！")
        # rospy.sleep(1)
        # rospy.loginfo("尝试识别标定物...")
        # rospy.wait_for_service('/object_detect')
        # detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
        # response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT)
        # if len(response.blueObjList)==1:
        #     print("识别到蓝色物体")
        #     x = response.blueObjList[0].position.x
        #     y = response.blueObjList[0].position.y
        #     blue_bin_positions = np.radians([119, -21, -28, 0, -82, 32]) # 目标
        #     control(x, y, blue_bin_positions)
        # if len(response.redObjList)==1:
        #     print("识别到红色物体")
        #     x = response.redObjList[0].position.x
        #     y = response.redObjList[0].position.y
        #     red_bin_positions = np.radians([90, -23, -33, 3, -79, 3]) # 目标                 
        #     control(x, y, red_bin_positions)
        # if len(response.greenObjList)==1:
        #     print("识别到绿色物体")
        #     x = response.greenObjList[0].position.x
        #     y = response.greenObjList[0].position.y
        #     green_bin_positions = np.radians([71, -21, -30, -8, -82, -25]) # 目标
        #     control(x, y, green_bin_positions)

        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass

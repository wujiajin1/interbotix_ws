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


class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        config_filename = '/home/ubuntu/interbotix_ws/src/object_color_detector/config/vision_config.yaml'

# 标定参数
        regObjList  = []
        # 初始化ROS节点
        rospy.init_node('grasp_mine', anonymous=True)

 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander("interbotix_arm")
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)
        
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)

        joint_positions = np.radians([0,-24,-52,0,-66,0]) # 目标
        arm.set_joint_value_target(joint_positions)                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        input_value = input("请将标定物放置到夹爪正下方，按任意键确认！")
        rospy.sleep(1)
        rospy.loginfo("尝试识别标定物...")
        rospy.wait_for_service('/object_detect')
        detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
        response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT)
        def control():
            joint_positions = np.radians([0, 0, -45, 0, -45, 0]) # 目标
            arm.set_joint_value_target(joint_positions)                 
            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)

            gripper.set_named_target('Closed')
            gripper.go()
            rospy.sleep(1)
        if len(response.blueObjList)==1:
            print("识别到蓝色物体")
            control()
            arm.set_named_target('Home')
            arm.go()
            joint_positions = np.radians([119, -21, -28, 0, -82, 32]) # 目标
            arm.set_joint_value_target(joint_positions)                 
            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
            
            gripper.set_named_target('Open')
            gripper.go()
            rospy.sleep(1)

        if len(response.redObjList)==1:
            print("识别到红色物体")
            control()
            
            arm.set_named_target('Home')
            arm.go()
            joint_positions = np.radians([90, -23, -33, 3, -79, 3]) # 目标
            arm.set_joint_value_target(joint_positions)                 
            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
            
            gripper.set_named_target('Open')
            gripper.go()
            rospy.sleep(1)
        if len(response.greenObjList)==1:
            print("识别到绿色物体")
            control()
            
            arm.set_named_target('Home')
            arm.go()
            joint_positions = np.radians([71, -21, -30, -8, -82, -25]) # 目标
            arm.set_joint_value_target(joint_positions)                 
            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
            
            gripper.set_named_target('Open')
            gripper.go()
            rospy.sleep(1)
        
        # 控制夹爪先回到初始化位置
        # 控制夹爪闭合
        # gripper.set_named_target('Closed')
        # gripper.go()
        # rospy.sleep(1)

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.1707, 0.0738, -0.5415, -0.1663, 0.0, 0.0]
        
        # joint_positions = np.radians([0, 6, -32, 0, -48, 0]) # 目标
        # arm.set_joint_value_target(joint_positions)                 
        # # 控制机械臂完成运动
        # arm.go()
        # rospy.sleep(1)


        # # 控制夹爪打开
        # gripper.set_named_target('Open')
        # gripper.go()
        # rospy.sleep(1)


        # # jia
        # joint_positions = np.radians([0, 6, -32, 0, -48, 0]) # 目标
        # arm.set_joint_value_target(joint_positions)                 
        # # 控制机械臂完成运动
        # arm.go()
        # rospy.sleep(1)


        # # 控制夹爪闭合
        # joint_positions = [0.025, -0.025] # small cube
        # gripper.set_joint_value_target(joint_positions)
        # # gripper.set_named_target('Closed')
        # gripper.go()
        # rospy.sleep(1)

        # # 控制机械臂先回到初始化位置
        # arm.set_named_target('Home')
        # # joint_positions = [0.0, -1.8000, -1.5500, 0.0, -0.8000, 0.0] # sleep
        # arm.go()
        # rospy.sleep(1)
        
        # joint_positions = np.radians([0, 6, -32, 0, -48, 0]) # 目标
        # arm.set_joint_value_target(joint_positions)                 
        # # 控制机械臂完成运动
        # arm.go()
        # rospy.sleep(1)
        
        # # 控制夹爪打开
        # gripper.set_named_target('Open')
        # gripper.go()
        # rospy.sleep(1)
        
        arm.set_named_target('Sleep')
        # joint_positions = [0.0, -1.8000, -1.5500, 0.0, -0.8000, 0.0] # sleep
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import copy

class PickAndPlace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('jaka_pick_place', anonymous=True)

        self.arm = moveit_commander.MoveGroupCommander("jaka_minicobo")
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        self.target_pose = None
        self.is_grasped = False

        # 订阅视觉坐标
        rospy.Subscriber("/target_pose", Pose, self.pose_callback)
        
        # 新增：发布 Gazebo 模型状态，用于强制实现“吸附”效果
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        # 新增：开启一个 50Hz 的后台定时器，不断更新物块位置
        rospy.Timer(rospy.Duration(0.02), self.grasp_loop)

        rospy.loginfo("Waiting for target pose from vision...")

    def pose_callback(self, msg):
        if self.target_pose is None:
            self.target_pose = msg
            rospy.loginfo(f"Target found at X: {msg.position.x:.3f}, Y: {msg.position.y:.3f}")

    def grasp_loop(self, event):
        # 如果处于“吸附”状态，强制物块跟随机械臂末端
        if self.is_grasped:
            try:
                curr_pose = self.arm.get_current_pose().pose
                block_state = ModelState()
                block_state.model_name = 'target_block'
                
                # 物块对齐机械臂末端，并在Z轴向下偏移一小段距离(物块中心点)，避免物理碰撞
                block_state.pose.position.x = curr_pose.position.x
                block_state.pose.position.y = curr_pose.position.y
                block_state.pose.position.z = curr_pose.position.z - 0.025
                
                block_state.pose.orientation = curr_pose.orientation
                block_state.reference_frame = 'world'
                self.model_state_pub.publish(block_state)
            except Exception as e:
                pass

    def execute_task(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.target_pose is None:
            rate.sleep()

        if self.target_pose is None:
            return

        rospy.loginfo("Step 1: Moving to pre-grasp position...")
        pre_grasp_pose = copy.deepcopy(self.target_pose)
        pre_grasp_pose.position.z += 0.15 
        pre_grasp_pose.orientation.x = 0.0
        pre_grasp_pose.orientation.y = 1.0
        pre_grasp_pose.orientation.z = 0.0
        pre_grasp_pose.orientation.w = 0.0
        
        self.arm.set_pose_target(pre_grasp_pose)
        self.arm.go(wait=True)

        rospy.loginfo("Step 2: Going down to grasp...")
        grasp_pose = copy.deepcopy(pre_grasp_pose)
        # 停在 Z=0.055，留出 5mm 间隙，防止法兰盘和方块发生刚性碰撞导致 Gazebo 物理引擎崩溃
        grasp_pose.position.z = 0.055 
        self.arm.set_pose_target(grasp_pose)
        self.arm.go(wait=True)
        
        rospy.sleep(0.5)
        
        # 触发吸附魔法！
        self.is_grasped = True 
        rospy.loginfo("==> Block GRASPED! (Attached to TCP)")

        rospy.loginfo("Step 3: Lifting object...")
        self.arm.set_pose_target(pre_grasp_pose)
        self.arm.go(wait=True)

        rospy.loginfo("Step 4: Moving to place position...")
        place_pose = copy.deepcopy(pre_grasp_pose)
        place_pose.position.x = 0.1  # 放置点 X
        place_pose.position.y = 0.3  # 放置点 Y
        self.arm.set_pose_target(place_pose)
        self.arm.go(wait=True)

        rospy.loginfo("Step 5: Going down to place...")
        place_down_pose = copy.deepcopy(place_pose)
        place_down_pose.position.z = 0.055
        self.arm.set_pose_target(place_down_pose)
        self.arm.go(wait=True)
        
        rospy.sleep(0.5)
        
        # 关闭吸附，物块将重新受到物理引擎的重力控制，安稳地留在桌面上
        self.is_grasped = False
        rospy.loginfo("==> Block RELEASED!")

        rospy.loginfo("Step 6: Returning home...")
        self.arm.set_pose_target(place_pose) 
        self.arm.go(wait=True)
        
        rospy.loginfo("Task Complete!")

if __name__ == '__main__':
    try:
        task = PickAndPlace()
        task.execute_task()
    except rospy.ROSInterruptException:
        pass

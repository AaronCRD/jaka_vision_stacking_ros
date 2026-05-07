#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import copy

class MultiStacking:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('jaka_multi_stacking', anonymous=True)
        self.arm = moveit_commander.MoveGroupCommander("jaka_minicobo")
        self.arm.set_max_velocity_scaling_factor(0.6)

        self.poses = {"red": None, "green": None, "blue": None}
        rospy.Subscriber("/target/red", Pose, lambda msg: self.poses.update({"red": msg}))
        rospy.Subscriber("/target/green", Pose, lambda msg: self.poses.update({"green": msg}))
        rospy.Subscriber("/target/blue", Pose, lambda msg: self.poses.update({"blue": msg}))

        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.current_attached_model = None
        # 控制周期改为 125Hz 进行高频状态同步
        rospy.Timer(rospy.Duration(1.0 / 125.0), self.grasp_loop)

    def grasp_loop(self, event):
        if self.current_attached_model:
            try:
                curr_pose = self.arm.get_current_pose().pose
                block_state = ModelState()
                block_state.model_name = self.current_attached_model
                block_state.pose.position.x = curr_pose.position.x
                block_state.pose.position.y = curr_pose.position.y
                block_state.pose.position.z = curr_pose.position.z - 0.025
                block_state.pose.orientation = curr_pose.orientation
                block_state.reference_frame = 'world'
                self.model_state_pub.publish(block_state)
            except:
                pass

    def pick_and_place(self, model_name, target_pose, place_z):
        rospy.loginfo(f"--- Processing {model_name} ---")

        # 1. 移动到抓取点上方
        pre_grasp = copy.deepcopy(target_pose)
        pre_grasp.position.z += 0.15 
        pre_grasp.orientation.x, pre_grasp.orientation.y = 0.0, 1.0
        pre_grasp.orientation.z, pre_grasp.orientation.w = 0.0, 0.0
        self.arm.set_pose_target(pre_grasp)
        self.arm.go(wait=True)

        # 2. 下降并吸附
        grasp_pose = copy.deepcopy(pre_grasp)
        grasp_pose.position.z = 0.055
        self.arm.set_pose_target(grasp_pose)
        self.arm.go(wait=True)
        self.current_attached_model = model_name
        rospy.sleep(0.5)

        # 3. 抬起并移动到堆叠中心 (X=0.1, Y=0.3)
        self.arm.set_pose_target(pre_grasp)
        self.arm.go(wait=True)

        place_pose = copy.deepcopy(pre_grasp)
        place_pose.position.x = 0.1
        place_pose.position.y = 0.3
        self.arm.set_pose_target(place_pose)
        self.arm.go(wait=True)

        # 4. 下降到指定的高程进行堆叠
        place_down = copy.deepcopy(place_pose)
        place_down.position.z = place_z
        self.arm.set_pose_target(place_down)
        self.arm.go(wait=True)

        # 松开
        self.current_attached_model = None
        rospy.sleep(0.5)

        # 安全抬起准备下一次抓取
        self.arm.set_pose_target(place_pose)
        self.arm.go(wait=True)

    def execute_sequence(self):
        rospy.loginfo("Waiting for vision data...")
        while not rospy.is_shutdown() and not all(self.poses.values()):
            rospy.Rate(10).sleep()

        # 堆叠高度计算：第一层 Z=0.055, 第二层=0.105, 第三层=0.145 (小方块)
        tasks = [
            ("red_box", self.poses["red"], 0.055),
            ("green_cylinder", self.poses["green"], 0.105),
            ("blue_box", self.poses["blue"], 0.145)
        ]

        for model_name, pose, z_height in tasks:
            self.pick_and_place(model_name, pose, z_height)

        rospy.loginfo("Stacking sequence completed successfully!")

if __name__ == '__main__':
    try:
        task = MultiStacking()
        task.execute_sequence()
    except rospy.ROSInterruptException:
        pass
        

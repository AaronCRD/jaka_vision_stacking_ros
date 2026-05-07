#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

class VisionDetector:
    def __init__(self):
        rospy.init_node('vision_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/top_camera/image_raw", Image, self.image_callback)
        self.target_pub = rospy.Publisher("/target_pose", Pose, queue_size=10)
        
        # 相机参数 (根据 URDF 中的设定)
        self.camera_z = 1.0 # 相机高度
        self.image_width = 800
        self.image_height = 800
        self.fov = 1.047 # 60度
        
        # 计算像素到物理尺寸的比例 (高度为1m时的视野宽度)
        self.view_width_m = 2.0 * self.camera_z * np.tan(self.fov / 2.0)
        self.pixel_to_m = self.view_width_m / self.image_width

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # 转换到 HSV 色彩空间寻找红色
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # 找到最大的红色轮廓
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # 在图像上画出中心点
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.imshow("Camera View", cv_image)
                cv2.waitKey(3)

                # --- 像素坐标到世界坐标的转换 ---
                # 图像中心是 (400, 400)，对应世界坐标 X=0.3, Y=0.0
                offset_x_pixels = cy - (self.image_height / 2) # 图像Y轴对应世界的X轴(由于相机旋转)
                offset_y_pixels = cx - (self.image_width / 2)  # 图像X轴对应世界的Y轴
                
                # Gazebo相机坐标系转换关系：图像的X向右，Y向下。
                # 我们的世界坐标系：正前是X，正左是Y。
                world_x = 0.3 - (offset_x_pixels * self.pixel_to_m)
                world_y = 0.0 - (offset_y_pixels * self.pixel_to_m)
                
                # 发布目标姿态
                pose = Pose()
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = 0.025 # 物块中心高度
                self.target_pub.publish(pose)

if __name__ == '__main__':
    try:
        vd = VisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

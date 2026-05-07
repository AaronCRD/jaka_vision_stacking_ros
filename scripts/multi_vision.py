#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose

class MultiVisionDetector:
    def __init__(self):
        rospy.init_node('multi_vision_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/top_camera/image_raw", Image, self.image_callback)

        # 分别为三种颜色创建发布者
        self.pub_red = rospy.Publisher("/target/red", Pose, queue_size=10)
        self.pub_green = rospy.Publisher("/target/green", Pose, queue_size=10)
        self.pub_blue = rospy.Publisher("/target/blue", Pose, queue_size=10)

        self.camera_z = 1.0
        self.image_size = 800
        self.pixel_to_m = (2.0 * self.camera_z * np.tan(1.047 / 2.0)) / self.image_size

    def detect_and_publish(self, hsv, cv_image, color_lower, color_upper, color_name, publisher):
        mask = cv2.inRange(hsv, color_lower, color_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

                # 绘制标记与文字
                cv2.circle(cv_image, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(cv_image, color_name, (cx-20, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # 坐标系转换
                world_x = 0.3 - ((cy - self.image_size / 2) * self.pixel_to_m)
                world_y = 0.0 - ((cx - self.image_size / 2) * self.pixel_to_m)

                pose = Pose()
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = 0.025
                publisher.publish(pose)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # HSV 颜色阈值
        self.detect_and_publish(hsv, cv_image, np.array([0, 100, 100]), np.array([10, 255, 255]), "Red", self.pub_red)
        self.detect_and_publish(hsv, cv_image, np.array([40, 50, 50]), np.array([80, 255, 255]), "Green", self.pub_green)
        self.detect_and_publish(hsv, cv_image, np.array([100, 100, 100]), np.array([140, 255, 255]), "Blue", self.pub_blue)

        cv2.imshow("Multi-Target Vision", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    vd = MultiVisionDetector()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_object(name, shape_xml, color, x, y):

    # 动态生成带有高摩擦力和防弹跳物理属性的 URDF
    urdf = f"""<?xml version="1.0"?>
    <robot name="{name}">
      <link name="link">
        <inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
        <visual><geometry>{shape_xml}</geometry></visual>
        <collision><geometry>{shape_xml}</geometry></collision>
      </link>
      <gazebo reference="link">
        <material>Gazebo/{color}</material>
        <mu1>100.0</mu1>
        <mu2>100.0</mu2>
        <kp>1000000.0</kp>
        <kd>100.0</kd>
        <minDepth>0.001</minDepth>
      </gazebo>
    </robot>
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.05

    # 调用 Gazebo 服务
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    spawn_model(name, urdf, "", pose, "world")
    rospy.loginfo(f"Spawned {name} at ({x}, {y})")

if __name__ == '__main__':
    rospy.init_node('spawn_multiple_objects')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    # 投放三个不同形状、不同颜色的物块
    spawn_object("red_box", "<box size='0.1 0.1 0.03'/>", "Red", 0.2, 0.15)
    spawn_object("green_cylinder", "<cylinder radius='0.04' length='0.03'/>", "Green", 0.35, 0.0)
    spawn_object("blue_box", "<box size='0.04 0.04 0.04'/>", "Blue", 0.3, 0.1)

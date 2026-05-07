![Demo](demo.gif)
JAKA Minicobo Vision Pick & Stacking 🤖👁️
基于 ROS Noetic 与 Gazebo 的手眼协同视觉抓取与物理堆叠仿真项目。

🌟 项目简介
本项目使用节卡 (JAKA) Minicobo 协作机械臂，结合顶置 RGB 相机，实现了从单目标颜色识别抓取，到多目标（红、绿、蓝）动态生成、识别及高稳定物理堆叠的端到端仿真。

✨ 核心特性：

动态环境生成：通过代码动态投放不同形状与颜色的 URDF 模型。

视觉与坐标转换：基于 OpenCV 的色块识别与世界坐标精确定位。

虚拟吸附技术：使用高频 ROS 定时器同步模型状态，完美模拟真实吸盘抓取。

稳定堆叠物理模型：调优 Gazebo 摩擦力与穿透参数，解决仿真堆叠滑落难题。

🛠️ 依赖环境
Ubuntu 20.04 + ROS Noetic

MoveIt 1

Python 3.x (OpenCV, cv_bridge)

jaka_robot 官方功能包 (需预先安装并编译)

🚀 快速运行
模式一：基础单目标抓取

1.启动环境：roslaunch jaka_vision_pick vision_pick.launch

2.启动视觉：rosrun jaka_vision_pick vision_detector.py

3.执行抓取：rosrun jaka_vision_pick control_node.py

模式二：多目标进阶堆叠 (Stacking)

1.启动环境：roslaunch jaka_vision_pick multi_vision_pick.launch

2.动态投放物块：rosrun jaka_vision_pick spawn_objects.py

3.启动多目标视觉：rosrun jaka_vision_pick multi_vision.py

4.执行自动堆叠：rosrun jaka_vision_pick multi_control.py

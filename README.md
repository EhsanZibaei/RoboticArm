This ROS2 project simulates a robotic arm performing an unscrewing scenario.
The screws are simulated as revolute joints on a box in Gazebo Harmonic, and the screwdriver_tcp is moved to the screw head using the MoveIt 2 library. Finally, gz_ros2_control is used to move the screwdriver head to touch the screw head. In the future, reinforcement learning methods will be used to perform precise unscrewing actions.


![Demo](resources/demo.gif)
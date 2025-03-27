rm -rf build/ install/ log/
colcon build --packages-select my_second_robot moving5
source install/setup.bash
# ros2 launch my_second_robot start_gz_ros2_control_from_template.launch.py
# ros2 launch my_second_robot add_moveit.launch.py
ros2 launch moving5 demo.launch.py
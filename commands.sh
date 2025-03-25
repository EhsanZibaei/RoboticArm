rm -rf build/ install/ log/
colcon build --packages-select my_second_robot
source install/setup.bash
# ros2 launch my_second_robot fake_state_publisher.launch.py
ros2 launch my_second_robot start_gz_ros2_control_from_template.launch.py
# ros2 launch moving3 demo.launch.py
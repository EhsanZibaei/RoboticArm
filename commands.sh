rm -rf build/ install/ log/
colcon build 
source install/setup.bash
# ros2 launch my_second_robot fake_state_publisher.launch.py
ros2 launch my_second_robot robot_in_gz.launch.py
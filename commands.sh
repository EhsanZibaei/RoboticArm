rm -rf build/ install/ log/
colcon build --packages-select my_second_robot moving5
source install/setup.bash
# ros2 launch my_second_robot fake_state_publisher.launch.py
# ros2 launch my_second_robot add_moveit.launch.py
ros2 launch moving5 demo.launch.py
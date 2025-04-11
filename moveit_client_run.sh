# rm -rf build/ install/ log/
colcon build --packages-select state_machine 
source install/setup.bash
# ros2 launch my_second_robot start_gz_ros2_control_from_template.launch.py
# ros2 launch moving6 only_moveit.launch.py
ros2 launch state_machine task_planner.launch.py
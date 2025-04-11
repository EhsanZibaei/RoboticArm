from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="unscrew_simulation"
        )
        .robot_description()
        .robot_description_semantic(file_path="config/circu.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml") 
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=False
        )
        .planning_pipelines(
            pipelines=[
                # "ompl", 
                # "stomp", 
                "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    robot_move_node = Node(
        name="task_planner_node",
        package="state_machine",
        executable="robot_move_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([robot_move_node])
ros2 topic pub /g1_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [{
    positions: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"


ros2 topic pub /screwdriver_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: ['screwdriver-screwdriver_tcp'],
  points: [{
    positions: [-0.4],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"
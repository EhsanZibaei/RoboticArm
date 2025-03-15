import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class JointMover(Node):
    def __init__(self):
        super().__init__("joint_mover")
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        self.angle = 0.0
        self.get_logger().info("Publishing fake joint states...")

    def publish_joint_state(self):
        """Publishes a fake joint state to move `joint_1` dynamically."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Joint names
        msg.name = [
            "joint_1", "joint_2", "joint_3", 
            "joint_4", "joint_5", "joint_6", 
            "piston_joint", "cylinder_joint"
        ]
        
        msg.position = [
            math.sin(time.time()),  # joint_1 oscillates
            0, 0, 0, 0, 0,  # Fixed joints
            0, 0  # Fixed piston and cylinder joints
        ]
        
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)

        self.get_logger().info(f"Setting Joint_1 Position: {msg.position[0]}")
        self.joint_state_publisher.publish(msg)

def main():
    rclpy.init()
    node = JointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

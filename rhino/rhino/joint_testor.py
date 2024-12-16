import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTestor(Node):
    def __init__(self):
        super().__init__('joint_testor')
        # Create a publisher for the 'arm_controller/joint_trajectory' topic
        self.publisher_ = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)  # Publish every second

    def publish_trajectory(self):
        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]  # Specify joint names

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -1.0, 0.5, -1.0]  # Example positions (radians or meters)
        point.time_from_start = Duration(sec=2, nanosec=0)  # Time to reach this point

        msg.points.append(point)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Published JointTrajectory message")


def main(args=None):
    rclpy.init(args=args)
    node = JointTestor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

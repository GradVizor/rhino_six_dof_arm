import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class ControlTest(Node):
    def __init__(self):
        super().__init__('control_test')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.move_joints)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6a']  # Example joint names
        self.get_logger().info("Robot chain and targets initialized.")


    
    def move_joints(self, joint_angles):
        # Create a JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # Create a JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = [0.5, 1.0, -0.5, 0.8, -1.2, 0.6]  # Example joint positions in radians
        point.velocities = [0.0] * len(self.joint_names)  # Optional: set velocities to zero
        point.time_from_start.sec = 2  # Move to target positions in 2 seconds

        traj_msg.points.append(point)

def main(args=None):
    rclpy.init(args=args)
    control = ControlTest()

    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        control.get_logger().info("Node interrupted by user.")
    finally:
        control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

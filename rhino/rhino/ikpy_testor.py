import os
import math
import rclpy
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain    # Added ikpy library
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# arm_controller :- publisher_1, msg1, point1
# gripper_controller :- publisher_2, msg2, point2

class IkpyTestor(Node):
    def __init__(self):
        super().__init__('ikpy_testor')
        self.publisher_1 = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.publisher_2 = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)

        # Getting the urdf from the package
        share_dir = get_package_share_directory('rhino')
        xacro_file = os.path.join(share_dir, 'urdf', 'robot.xacro.urdf')

        # Defining the kinematic chain
        self.chain = Chain.from_urdf_file(xacro_file)

        self.fingers_closed = [-1.300, -1.300] # Values of joint6a & joint6b to close the fingers.
        self.fingers_opened = [1.135, 1.135]   # Values of joint6a & joint6b to open the fingers.

    def publish_trajectory(self):

        # Target end-effector position
        target_position = [1.0, 0.0, 0.0]  # x, y, z in meters

        # Computing inverse kinematics to get joint angles
        joint_angles = self.chain.inverse_kinematics(target_position)

        # Creating a JointTrajectory message
        msg1 = JointTrajectory()
        msg1.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]  # Specifying joint names
        msg2 = JointTrajectory()
        msg2.joint_names = ["joint6a", "joint6b"]  # Specifying joint names

        try:
            valid_positions = [float(angle) if not (math.isnan(angle) or math.isinf(angle)) else 0.0 for angle in joint_angles[1:len(msg1.joint_names) + 1]]
        except Exception as e:
            self.get_logger().error(f"Error processing joint angles: {e}")
            return
        
        # Create a trajectory point
        point1 = JointTrajectoryPoint()
        point1.positions = valid_positions  # Exclude root joint and ensure matching size
        point1.time_from_start = Duration(sec=2, nanosec=0)  # Time to reach this point
        msg1.points.append(point1)

        # Create a trajectory point
        point2 = JointTrajectoryPoint()
        point2.positions = self.fingers_closed 
        point2.time_from_start = Duration(sec=1, nanosec=0) 
        msg2.points.append(point2)

        # Validate sizes before publishing
        if len(msg1.joint_names) != len(point1.positions):
            self.get_logger().error("Mismatch between joint_names and positions size")
            return

        # Publish the message
        self.publisher_1.publish(msg1)
        self.publisher_2.publish(msg2)

        self.get_logger().info(f"Published JointTrajectory with positions: {joint_angles[1:6]}")

def main(args=None):
    rclpy.init(args=args)
    node = IkpyTestor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

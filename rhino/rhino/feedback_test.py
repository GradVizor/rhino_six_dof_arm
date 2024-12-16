'''     In this Node I'm trying to establish a feedback mechanism 
        by subscribing to /joint_states and publishing further to 
        by using forward and inverse kinematics.                        '''

import os
import math
import time 
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain    # Added ikpy library
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

# arm_controller :-         publisher_1, msg1, point1
# gripper_controller :-     publisher_2, msg2, point2 

class FeedbackTest(Node):
    def __init__(self):
        super().__init__('feedback_test')

        # Publishers
        self.publisher_1 = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.publisher_2 = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publisher_callback)

        # Subscriber
        self.subscriber = self.create_subscription(JointState, 'joint_states', self.subscriber_callback, 10)

        # Defining the gripper states
        self.gripper_closed = [-1.300, -1.300] # Values of joint6a & joint6b to close the fingers.
        self.gripper_opened = [1.135, 1.135]   # Values of joint6a & joint6b to open the fingers.

        # Getting the urdf from the package
        share_dir = get_package_share_directory('rhino')
        xacro_file = os.path.join(share_dir, 'urdf', 'robot.xacro.urdf')
        self.chain = Chain.from_urdf_file(xacro_file, active_links_mask=[False, True, True, True, True, True, True])

        # Defining a list of points to achieve with end-effector
        self.waypoint=[1,2,3]
        self.joint_state_content = {}
        self.current_joint_angles = list(self.joint_state_content.values())

    def publisher_callback(self):
        threads = []
        threads.append(threading.Thread(target=self.arm_action, args=(0,)))
        threads.append(threading.Thread(target=self.gripper_action, args=(self.gripper_closed,)))
        actions = ["1) End-effector to 1st waypoint. ", "2) Gripper closed. "]
        
        # Start each thread with a delay
        for i, thread in enumerate(threads):
            self.get_logger().info(f"Published JointTrajectory with positions: {actions[i]}")
            thread.start()
            thread.join()
            time.sleep(3)  # Delay before starting the next thread

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        self.get_logger().info(f"All functions have been completed.")

    def subscriber_callback(self, msg_sub):
        for i in range(len(msg_sub.name)):  # Iterate over the joint names
            joint_name = msg_sub.name[i]  # Get the joint name
            joint_position = msg_sub.position[i]  # Get the corresponding position
            self.joint_state_content[joint_name] = joint_position  # Update dictionary

        
    def arm_action(self, i):

        self.current_position = self.chain.forward_kinematics(self.current_joint_angles)
        joint_angles = self.chain.inverse_kinematics(self.waypoint, self.current_position)
        
        # Creating a JointTrajectory message for "arm"
        msg1 = JointTrajectory()
        msg1.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]

        try:
            valid_positions = [float(angle) if not (math.isnan(angle) or math.isinf(angle)) else 0.0 for angle in joint_angles[1:len(msg1.joint_names) + 1]]
        except Exception as e:
            self.get_logger().error(f"Error processing joint angles: {e}")
            return
        
        # Creating a trajectory point for "arm"
        point1 = JointTrajectoryPoint()
        point1.positions = valid_positions  # Exclude root joint and ensure matching size
        point1.time_from_start = Duration(sec=2, nanosec=0)  # Time to reach this point
        msg1.points.append(point1)

        # Validating sizes before publishing
        if len(msg1.joint_names) != len(point1.positions):
            self.get_logger().error("Mismatch between joint_names and positions size")
            return
        
        self.publisher_1.publish(msg1)
    
    def gripper_action(self, action):
        # Creating a JointTrajectory message for "gripper"
        msg2 = JointTrajectory()
        msg2.joint_names = ["joint6a", "joint6b"]

        # Create a trajectory point for "gripper"
        point2 = JointTrajectoryPoint()
        point2.positions = action
        point2.time_from_start = Duration(sec=1, nanosec=0) 
        msg2.points.append(point2)

        self.publisher_2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

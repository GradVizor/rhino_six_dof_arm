'''     In this Node I've combined 5 actions:- 
            1) Moving End-effector from initial to point1
            2) Moving End-effector from point1 to point2
            3) Closing the gripper. 
            4) Moving End-effector from point2 to point3
            4) Opening the gripper.                             
            
        Note:- Orientation is also  controlled               '''

import os
import math
import time 
import rclpy
import threading
from rclpy.node import Node
from ikpy.chain import Chain   
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Task(Node):
    def __init__(self):
        super().__init__('task2')

        # Publishers
        self.publisher_1 = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.publisher_2 = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publisher_callback)

        # Defining the gripper states
        self.gripper_closed = [-1.02, -1.02] # Values of joint6a & joint6b to close the fingers.
        self.gripper_opened = [0.0, 0.0]   # Values of joint6a & joint6b to open the fingers.

        # Getting the urdf from the package
        share_dir = get_package_share_directory('rhino_description')
        xacro_file = os.path.join(share_dir, 'description', 'urdf', 'robot.urdf')
        self.chain = Chain.from_urdf_file(xacro_file, active_links_mask=[False, True, True, True, True, True, True])

        # Defining a list of points to achieve with end-effector
        self.waypoints=[[0.408, -0.02, 0.4], [0.408, -0.02, 0.07], [-1,-2,0.07]]

    def publisher_callback(self):
        threads = []
        threads.append(threading.Thread(target=self.arm_action, args=(0,)))
        threads.append(threading.Thread(target=self.arm_action, args=(1,)))
        threads.append(threading.Thread(target=self.gripper_action, args=(self.gripper_closed,)))
        threads.append(threading.Thread(target=self.arm_action, args=(2,)))
        threads.append(threading.Thread(target=self.gripper_action, args=(self.gripper_opened,)))
        
        actions = ["1) End-effector to 1st waypoint. ", "2) End-effector to 2nd waypoint. ", 
                   "3) Gripper closed. ", "4) End-effector to 3rd waypoint. " , "5) Gripper opened. " ]
        
        # Start each thread with a delay
        for i, thread in enumerate(threads):
            self.get_logger().info(f"Published JointTrajectory with positions: {actions[i]}")
            thread.start()
            thread.join()
            time.sleep(6)  # Delay before starting the next thread

        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        self.get_logger().info(f"All actions have been completed.")
        
    def arm_action(self, i):
        
        # Defining the target position and orientation and calculating the Inverse Kinematics 
        target_position = self.waypoints[i]
        target_orientation = [0, 1, 0]
        joint_angles = self.chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z")
        
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
        point1.time_from_start = Duration(sec=3, nanosec=0)  # Time to reach this point
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

        # Creating a trajectory point for "gripper"
        point2 = JointTrajectoryPoint()
        point2.positions = action
        point2.time_from_start = Duration(sec=2, nanosec=0) 
        msg2.points.append(point2)

        self.publisher_2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = Task()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

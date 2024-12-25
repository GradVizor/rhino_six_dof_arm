import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # =========== Launch Arguments ============
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    share_dir = os.path.join(get_package_share_directory('rhino_description'))


    # ============= Launch Nodes ==============
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rhino_bringup'),
                'launch',
                'gz_sim.launch.py',)))

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"])

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')
    
    # joint_state_broadcaster --> arm_controller
    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],))

    # arm_controller --> gripper_controller
    delay_gripper_controller_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller],))
        
    sim_true = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    
    return LaunchDescription([
        gazebo,
        sim_true,
        ros_gz_bridge,
        joint_state_broadcaster,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_gripper_controller_after_arm_controller
    ])

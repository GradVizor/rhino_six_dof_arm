from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    # Declare arguments
    # declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "gui",
    #         default_value="true",
    #         description="Start RViz2 automatically with this launch file.",
    #     )
    # )
    share_dir = get_package_share_directory('rhino')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = robot_description_config.toxml()
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rhino"),
            "config",
            "rhino_controller.yaml",
        ]
    )
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("rhino"), "config", "display.rviz"]
    # )
    # rviz_config_file = "/home/dev_ws/src/rhino/config/display.rviz"

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # gui = LaunchConfiguration("gui")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rhino_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Delay start of joint_state_broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        # rviz_node,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
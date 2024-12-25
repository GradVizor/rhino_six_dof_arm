from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # =========== Launch Arguments ============
    
    share_dir = get_package_share_directory('rhino_description')

    xacro_file = os.path.join(share_dir, 'description', 'urdf', 'robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    world = [os.path.join(share_dir, 'worlds', 'med_object_world.sdf')] #


    # ============= Launch Nodes ==============

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir,
                    'launch',
                    'rsp.launch.py',)))

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                                'launch', 'ign_gazebo.launch.py')]),
                        launch_arguments=[('gz_args', [' -r -v 4 ']+ world)])

    rhino_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_urdf,
                   '-name', 'rhino',
                   '-allow_renaming', 'true'],)
    

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(share_dir,
            'description', 'models'))

    return LaunchDescription([
        set_env_vars_resources,
        rsp,
        gazebo,
        rhino_spawner
    ])
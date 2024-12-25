from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # =========== Launch Arguments ============
    
    share_dir = get_package_share_directory('rhino_description')
    # Select the type of object you wanna spawn. 
    object_sdf_file = os.path.join(share_dir, 'description', 'models', 'med_object', 'model.sdf')
    
    
    # ============= Launch Nodes ==============
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
            os.path.join(share_dir,
                'description', 'models'))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir,
                'launch',
                'rsp.launch.py',)))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_ign_gazebo'),
        'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]) 

    object_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'object',
            '-file', object_sdf_file,
            '-x', '-0.45',
            '-y', '0.086935',
            '-z', '0'],
            output='screen',)

    return LaunchDescription([
        set_env_vars_resources,
        # rsp,
        gazebo,
        object_spawner
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    # Get package paths
    skyforge_pkg = get_package_share_directory('skyforge')
    xacro_path = os.path.join(skyforge_pkg, 'urdf', 'skyforge_robot.urdf.xacro')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Convert .xacro to URDF
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Could not find xacro file at: {xacro_path}")

    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    # Write URDF to temp location for gazebo to read
    urdf_temp_path = os.path.join('/tmp', 'skyforge_robot.urdf.xacro')
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            )
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_skyforge',
            arguments=['-name', 'skyforge', '-file', urdf_temp_path],
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    # Paths
    package_dir = get_package_share_directory('skyforge')
    urdf_xacro_path = os.path.join(package_dir, 'urdf', 'skyforge_robot.urdf.xacro')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Convert xacro to URDF
    robot_description_config = xacro.process_file(urdf_xacro_path)
    robot_description = robot_description_config.toxml()

    # Save to temp for Gazebo to spawn
    urdf_temp_path = os.path.join('/tmp', 'skyforge_robot.urdf.xacro')
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),

        # Launch Gazebo (Garden/Fortress)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            )
        ),

        # Spawn the robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'skyforge', '-file', urdf_temp_path],
            output='screen',
        ),
    ])

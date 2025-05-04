from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package path
    skyforge_pkg = get_package_share_directory('skyforge')

    # Process Xacro
    xacro_path = os.path.join(skyforge_pkg, 'urdf', 'skyforge_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    # Write URDF to temp file
    urdf_temp_path = '/tmp/skyforge_robot.urdf'
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)

    # Controller config
    controller_yaml_path = os.path.join(skyforge_pkg, 'config', 'skyforge_controllers.yaml')

    return LaunchDescription([
        # Set Gazebo resource path
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(skyforge_pkg)
        ),

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '--render-engine', 'ogre2'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_skyforge',
            arguments=[
                '-name', 'skyforge',
                '-file', urdf_temp_path,
                '-z', '2.0',
                '--verbose'
            ],
            output='screen'
        ),

        # ros2_control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_yaml_path
            ],
            output='screen'
        ),

        # Delay and then spawn joint_state_broadcaster
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager', '/controller_manager'
                    ],
                    output='screen'
                ),
            ]
        ),

        # Delay and then spawn position controller
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'arm_controllers',
                        '--controller-manager', '/controller_manager'
                    ],
                    output='screen'
                ),
            ]
        ),
    ])

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('four_diff_drive_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'four_diff_drive.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    ign_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': 'false'
        }.items()
    )

    urdf_spawn_node = Node(
        package='ros_ign_gazebo',
        executable='spawn_entity',
        name='spawn_entity',
        arguments=[
            '-entity', 'four_diff_drive',
            '-file', os.path.join(share_dir, 'urdf', 'four_diff_drive.urdf'),  # Change to .urdf if necessary
            '-z', '1.0',  # Adjust this to change the height (z position) in meters
        ],
        output='screen'
    )

    spawner_controller_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_group_velocity_controller'],
                output='screen'
            )
        ]
    )

    robot_mover = Node(
        package='four_diff_drive_description',  # Replace with the actual package name
        executable='robot_mover',  # Replace with the actual name of your Python file (without .py)
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        ign_gazebo_server,
        urdf_spawn_node,
        spawner_controller_node,
        robot_mover,  
    ])

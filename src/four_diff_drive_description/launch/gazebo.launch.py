from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    share_dir = get_package_share_directory('four_diff_drive_description')
    
    # Path to the URDF file
    # urdf_path = os.path.join(share_dir, 'urdf', 'four_diff_drive.urdf')

    # # Load the URDF content for robot_state_publisher
    # with open(urdf_path, 'r') as urdf_file:
    #     urdf_content = urdf_file.read()

    xacro_file = os.path.join(share_dir, 'urdf', 'four_diff_drive.xacro')
    urdf_content = xacro.process_file(xacro_file).toxml()

    world_name = LaunchConfiguration('world_name', default='four_diff_drive_world')
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=[
            os.path.join("/opt/ros/humble", "share"),
            ":" + os.path.join(get_package_share_directory('four_diff_drive_description'), "models")
        ]
    )

    # Declare use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('four_diff_drive_description'),
                        "models", "worlds", "world_model.sdf"]),
                   '-allow_renaming', 'false'],
    )
    
    world_only = os.path.join(get_package_share_directory('four_diff_drive_description'), "models", "worlds", "world_only.sdf")

    ign_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gz_args': '-r -v 3 ' + world_only,
        }.items()
    )

    spawn_entity_service = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_entity',
        arguments=[
            '--name', 'four_diff_drive',
            #'--file', xacro_file,
            '-topic', 'robot_description',
            '--z', '1.0'
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
        package='four_diff_drive_description',
        executable='robot_mover',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', os.path.join(share_dir, 'config', 'lidar.rviz')],
    )

    return LaunchDescription([
        use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ign_gazebo_server,
        ign_resource_path,
        spawn_entity_service,
        spawner_controller_node,
        robot_mover,
        #rviz_node,

        IncludeLaunchDescription(PythonLaunchDescriptionSource([share_dir + '/launch/ros_ign_bridge.launch.py']),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                                 ),

        # IncludeLaunchDescription(PythonLaunchDescriptionSource([share_dir + '/launch/online_async_slam_toolbox.launch.py']),
        #                          launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        #                          ),
        
        IncludeLaunchDescription(PythonLaunchDescriptionSource([share_dir + '/launch/nav2.launch.py']),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                                 ),
    ])
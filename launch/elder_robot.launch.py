import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_share = get_package_share_directory('elder_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    urdf_file = os.path.join(pkg_share, 'urdf', 'elderbot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_config.rviz')

    '''
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    if not os.path.exists(world_file):
        world_file = os.path.join(
            os.environ['GAZEBO_RESOURCE_PATH'].split(':')[0],
            'worlds', 
            'empty.world'
        )
    '''

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time
        }.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=[
            '-entity', 'elderbot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    #Init SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        node_executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          slam_config_file,
          {'use_sim_time': use_sim_time}
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        gazebo,
        robot_state_pub,
        delayed_spawn,
        slam_toolbox,
        rviz2
    ])

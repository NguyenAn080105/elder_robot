import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('elder_robot')

    urdf_file = os.path.join(pkg_share, 'urdf', 'elderbot.urdf')
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    if not os.path.exists(world_file):
        world_file = os.path.join(
            os.environ['GAZEBO_RESOURCE_PATH'].split(':')[0],
            'worlds', 
            'empty.world'
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
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
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        delayed_spawn
    ])

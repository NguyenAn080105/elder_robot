from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('elder_robot')

    urdf_file = os.path.join(pkg_share, 'urdf', 'elderbot.urdf')
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    if not os.path.exists(world_file):
        world_file = os.path.join(os.environ['GAZEBO_RESOURCE_PATH'].split(':')[0], 'worlds', 'empty.world')

    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    robot_state_pub = ExecuteProcess(
        cmd=['/opt/ros/dashing/lib/robot_state_publisher/robot_state_publisher', urdf_file],
        output='screen'
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'elderbot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1', '-Y', '0'
        ],
        output='screen'
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    return LaunchDescription([
        start_gazebo,
        robot_state_pub,
        delayed_spawn
    ])

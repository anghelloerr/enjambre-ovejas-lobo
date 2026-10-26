"""Demo completa: gemelo digital en Gazebo + algoritmo ovejas-lobo.

  ros2 launch swarm_sheep_wolf demo.launch.py gui:=true
  ros2 launch swarm_sheep_wolf demo.launch.py wolf_mode:=hunt n_sheep:=3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = os.path.join(get_package_share_directory('microros_gazebo'), 'launch', 'sim.launch.py')
    swarm = os.path.join(get_package_share_directory('swarm_sheep_wolf'), 'launch', 'swarm.launch.py')
    args = ['n_sheep', 'gui', 'rviz', 'use_lidar', 'wolf_mode', 'pose_source', 'output', 'duration']
    lc = {a: LaunchConfiguration(a) for a in args}
    return LaunchDescription([
        DeclareLaunchArgument('n_sheep', default_value='5'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument('wolf_mode', default_value='herd'),
        DeclareLaunchArgument('pose_source', default_value='ground_truth'),
        DeclareLaunchArgument('output', default_value='~/swarm_logs/swarm_metrics.csv'),
        DeclareLaunchArgument('duration', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
            'n_sheep': lc['n_sheep'], 'gui': lc['gui'], 'rviz': lc['rviz'],
            'use_lidar': lc['use_lidar']}.items()),
        # Esperar a que los robots aparezcan y el EKF arranque
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(swarm), launch_arguments={
                'n_sheep': lc['n_sheep'], 'wolf_mode': lc['wolf_mode'],
                'pose_source': lc['pose_source'], 'output': lc['output'],
                'duration': lc['duration']}.items())]),
    ])

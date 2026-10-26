"""Lanza un agent_node por robot (ovejas + lobo) y el registro de metricas.

  ros2 launch swarm_sheep_wolf swarm.launch.py n_sheep:=5 wolf_mode:=herd pose_source:=ground_truth
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    n = int(LaunchConfiguration('n_sheep').perform(context))
    mode = LaunchConfiguration('wolf_mode').perform(context)
    source = LaunchConfiguration('pose_source').perform(context)
    out = LaunchConfiguration('output').perform(context)
    duration = float(LaunchConfiguration('duration').perform(context))
    cfg = os.path.join(get_package_share_directory('swarm_sheep_wolf'), 'config', 'swarm.yaml')
    sheep = [f'sheep{i + 1}' for i in range(n)]
    common = {'sheep_names': sheep, 'wolf_name': 'wolf', 'pose_source': source,
              'wolf_mode': mode, 'use_sim_time': True}
    nodes = [Node(package='swarm_sheep_wolf', executable='agent_node', name='agent', namespace=ns,
                  parameters=[cfg, dict(common, robot_name=ns, role='sheep')], output='screen')
             for ns in sheep]
    nodes.append(Node(package='swarm_sheep_wolf', executable='agent_node', name='agent', namespace='wolf',
                      parameters=[cfg, dict(common, robot_name='wolf', role='wolf')], output='screen'))
    nodes.append(Node(package='swarm_sheep_wolf', executable='metrics_node', output='screen',
                      parameters=[{'sheep_names': sheep, 'wolf_name': 'wolf', 'output': out,
                                   'duration': duration, 'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_sheep', default_value='5'),
        DeclareLaunchArgument('wolf_mode', default_value='herd'),
        DeclareLaunchArgument('pose_source', default_value='ground_truth'),
        DeclareLaunchArgument('output', default_value='~/swarm_logs/swarm_metrics.csv'),
        DeclareLaunchArgument('duration', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])

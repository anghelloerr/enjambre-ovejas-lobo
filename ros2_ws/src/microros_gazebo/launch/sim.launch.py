"""Gemelo digital multirrobot: N ovejas + 1 lobo MicroROS-Pi5 en Gazebo Fortress.

Por cada robot <ns> (sheep1..sheepN, wolf) se lanza:
  * el modelo xacro con prefijo <ns>/ y topicos /<ns>/{cmd_vel,odom_raw,imu,scan,ground_truth}
  * robot_state_publisher en el namespace <ns>
  * robot_localization (EKF) que publica /<ns>/odom y la TF <ns>/odom -> <ns>/base_footprint
  * una TF estatica world -> <ns>/odom con la pose de aparicion
Un unico ros_gz_bridge conecta todos los topicos.

Ejemplos
  ros2 launch microros_gazebo sim.launch.py
  ros2 launch microros_gazebo sim.launch.py n_sheep:=3 gui:=true use_lidar:=true
"""
import math
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHEEP_COLOR = '0.95 0.95 0.95 1'
WOLF_COLOR = '0.55 0.15 0.15 1'


def spawn_poses(n_sheep):
    """Rebano en rejilla alrededor de (-0.6, -0.6); lobo en (-1.6, -1.6)."""
    poses = []
    cols = math.ceil(math.sqrt(n_sheep))
    for i in range(n_sheep):
        r, c = divmod(i, cols)
        poses.append((f'sheep{i + 1}', -0.95 + 0.4 * c, -0.95 + 0.4 * r, 0.3 * i))
    poses.append(('wolf', -1.6, -1.6, math.pi / 4))
    return poses


def launch_setup(context):
    desc = get_package_share_directory('microros_description')
    gz = get_package_share_directory('microros_gazebo')
    n = int(LaunchConfiguration('n_sheep').perform(context))
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    mu_lat = LaunchConfiguration('mu_lat').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world = os.path.join(gz, 'worlds', 'arena.sdf')

    gz_args = ['ign', 'gazebo', '-r', '-v', '2', world]
    if not gui:
        gz_args.insert(2, '-s')
    actions = [ExecuteProcess(cmd=gz_args, output='screen')]

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    tmp = tempfile.mkdtemp(prefix='microros_')
    for ns, x, y, yaw in spawn_poses(n):
        color = WOLF_COLOR if ns == 'wolf' else SHEEP_COLOR
        urdf = xacro.process_file(os.path.join(desc, 'urdf', 'microros.urdf.xacro'),
                                  mappings={'prefix': f'{ns}/', 'ns': ns, 'color': color,
                                            'use_lidar': use_lidar, 'mu_lat': mu_lat}).toxml()
        path = os.path.join(tmp, f'{ns}.urdf')
        with open(path, 'w') as f:
            f.write(urdf)

        actions.append(Node(package='ros_gz_sim', executable='create', output='screen',
                            arguments=['-name', ns, '-file', path, '-x', str(x), '-y', str(y),
                                       '-z', '0.01', '-Y', str(yaw)]))
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=ns,
                            parameters=[{'robot_description': urdf, 'use_sim_time': True}],
                            remappings=[('joint_states', f'/{ns}/joint_states')]))
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                                       '--frame-id', 'world', '--child-frame-id', f'{ns}/odom']))
        actions.append(Node(package='robot_localization', executable='ekf_node',
                            name='ekf_filter_node', namespace=ns,
                            parameters=[os.path.join(gz, 'config', 'ekf.yaml'),
                                        {'odom_frame': f'{ns}/odom',
                                         'base_link_frame': f'{ns}/base_footprint',
                                         'world_frame': f'{ns}/odom',
                                         'odom0': f'/{ns}/odom_raw',
                                         'imu0': f'/{ns}/imu'}],
                            remappings=[('odometry/filtered', f'/{ns}/odom')]))
        bridge_args += [
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/{ns}/odom_raw@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/{ns}/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/{ns}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            f'/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ]
        if use_lidar == 'true':
            bridge_args.append(f'/{ns}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan')

    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=bridge_args, output='screen'))
    return actions


def generate_launch_description():
    desc = get_package_share_directory('microros_description')
    return LaunchDescription([
        DeclareLaunchArgument('n_sheep', default_value='5'),
        DeclareLaunchArgument('use_lidar', default_value='false',
                              description='LiDAR MS200 simulado (requiere GPU/ogre2)'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mu_lat', default_value='1.0',
                              description='friccion lateral de las ruedas (ver microros.urdf.xacro)'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(desc)),
        OpaqueFunction(function=launch_setup),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(get_package_share_directory('microros_gazebo'),
                                           'config', 'swarm.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])

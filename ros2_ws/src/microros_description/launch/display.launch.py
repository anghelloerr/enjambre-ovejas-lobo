"""Visualiza un MicroROS-Pi5 en RViz (equivalente a display_launch.py de Yahboom)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('microros_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'microros.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'microros.rviz'])]),
    ])

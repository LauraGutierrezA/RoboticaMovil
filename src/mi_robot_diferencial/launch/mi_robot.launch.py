import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'pista_pro.sdf')
    # world_path = os.path.join(pkg, 'worlds', 'pista.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    # El URDF generado por xacro se pasa como string, no como YAML
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    return LaunchDescription([
        # Gazebo con la pista, arrancando sin pausa (-r)
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', world_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'mi_robot',
            '-x', '-1.06', '-y', '1.06', '-z', '0.1', '-Y', '-0.785']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}]
        ),
    ])
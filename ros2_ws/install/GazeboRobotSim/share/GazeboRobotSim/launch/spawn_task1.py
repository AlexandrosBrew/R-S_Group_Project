import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('GazeboRobotSim'), 'models', 'robotTask1.sdf'
    )

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose'],
            output='screen'
        ),

        # Spawn the robot model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot_dimithri', '-file', model_path],
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    '''Launches the task3 world with the model.'''
    world_path = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share', 'GazeboRobotSim', 'worlds', 'task3.world'
        )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])

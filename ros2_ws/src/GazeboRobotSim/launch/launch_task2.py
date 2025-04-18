from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'GazeboRobotSim', 'worlds', 'task2.world'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        )
    ])

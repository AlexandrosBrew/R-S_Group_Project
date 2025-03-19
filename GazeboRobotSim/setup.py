from setuptools import find_packages, setup  # Import setup correctly
import os
from glob import glob  # Ensure glob is imported

package_name = 'GazeboRobotSim'  # Keeping the package name as requested

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Find packages except 'test'
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Ensure launch files are installed
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),  # Ensure model files are installed
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),  # Ensure script files are installed
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),  # Ensure script files are installed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dimithri',
    maintainer_email='dimithriweera@gmail.com',
    description='Gazebo Robot Simulation in ROS2',
    license='Apache License 2.0',  # Replace 'TODO' with actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = GazeboRobotSim.move_robot:main',
            'test_gazebo = GazeboRobotSim.test_gazebo:main',
        ],
    },
)

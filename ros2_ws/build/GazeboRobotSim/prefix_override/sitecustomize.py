import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/795302/R-S_Group_Project/ros2_ws/install/GazeboRobotSim'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eyad/rt_ws/src/ros2_robot_mover_python/install/ros2_robot_mover_python'

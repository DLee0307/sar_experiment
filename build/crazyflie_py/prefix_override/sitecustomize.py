import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dlee/ros2_ws/src/sar_experiment/install/crazyflie_py'

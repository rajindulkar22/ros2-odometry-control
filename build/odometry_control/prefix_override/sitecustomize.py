import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raj/raj_prac_ws/src/odometry_control/install/odometry_control'

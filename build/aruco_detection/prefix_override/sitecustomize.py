import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/RoverTech/nav2_ws_new/install/aruco_detection'

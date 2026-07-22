import sys
if sys.prefix == '/Users/giuliomastromartino/Documents/Polispace/ros2_humble':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/giuliomastromartino/Documents/Polispace/Nav2/install/aruco_detection_dev'

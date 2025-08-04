import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaeheyoung/Rokey/7.PracticalProject/Week8_Intel2/turtlebot3_ws/install/turtlebot3_teleop'

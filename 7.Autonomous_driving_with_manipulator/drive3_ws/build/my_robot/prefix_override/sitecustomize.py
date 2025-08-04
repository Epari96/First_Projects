import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaeheyoung/Rokey/7.PracticalProject/Week7_Drive3/drive3_ws/install/my_robot'

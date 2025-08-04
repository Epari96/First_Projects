import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaeheyoung/Rokey/7.PracticalProject/Week6_Drive2/rbvacuum_ws/install/detector'

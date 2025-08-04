import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaeheyoung/Rokey/7.PracticalProject/Week3_Coop2/DR_ws/install/rokey'

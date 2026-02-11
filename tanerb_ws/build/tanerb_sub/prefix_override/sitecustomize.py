import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ougy/damiao_ws/tanerb_ws/install/tanerb_sub'

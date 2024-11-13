import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/orinnano/Desktop/IAST_stuff_2_test/install/four_diff_drive_description'

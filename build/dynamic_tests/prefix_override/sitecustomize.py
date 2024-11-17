import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/filippo/Formula/dynamic_tests/install/dynamic_tests'

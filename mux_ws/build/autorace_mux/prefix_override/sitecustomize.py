import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/a/2026-AutoRace/mux_ws/install/autorace_mux'

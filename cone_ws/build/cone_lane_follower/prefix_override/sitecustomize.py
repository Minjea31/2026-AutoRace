import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wego/2026-AutoRace/cone_ws/install/cone_lane_follower'

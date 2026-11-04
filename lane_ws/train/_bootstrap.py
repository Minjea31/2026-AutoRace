"""Allow scripts to run from lane_ws/train before colcon installation."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'autorace_lane_lite'))

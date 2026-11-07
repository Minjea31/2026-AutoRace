from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(ROOT / 'simul/src/limo_parking_sim'))

sys.path.insert(0, str(ROOT / 'sensor_ws/src/autorace_sensor_bringup/scripts'))

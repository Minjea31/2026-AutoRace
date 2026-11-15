"""Preview measurements, validation, and read-only bag navigation contracts."""
import sqlite3

import cv2
import numpy as np
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image

from roi_preview import WhiteROI, defaults, render_preview
from roi_bag import BagImages


def encoded(frame):
    ok, data = cv2.imencode('.png', frame)
    assert ok
    return data.tobytes()


def test_preview_uses_actual_detector_original_resolution_and_pixel_bounds():
    params = defaults()['missions']['crosswalk']
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[413:442, 128:320] = 255
    result = render_preview(encoded(frame), params)
    detector = WhiteROI(**{key: params['white_roi_' + key] for key in ('left','right','top','bottom','minimum_ratio')})
    ratio, bounds, mask = detector.measure(frame, params['white_s_max'], params['white_v_min'])
    assert result['ratio'] == ratio
    assert result['bounds'] == list(bounds) == [128, 412, 512, 442]
    assert result['white_pixels'] == int(np.count_nonzero(mask))
    assert result['roi_pixels'] == int(mask.size)
    # Moving to an empty upper strip must change the result, not only the overlay.
    moved = {**params, 'white_roi_top': .1, 'white_roi_bottom': .2}
    assert render_preview(encoded(frame), moved)['ratio'] == 0
    assert result['ratio'] > 0


@pytest.mark.parametrize('changes', [
    {'white_roi_left': .9, 'white_roi_right': .2},
    {'white_roi_bottom': float('nan')}, {'white_roi_left': True},
    {'white_roi_minimum_ratio': 0}, {'white_s_max': 256}, {'white_v_min': 120.5}])
def test_invalid_parameters_rejected(changes):
    with pytest.raises(ValueError):
        render_preview(encoded(np.zeros((20,20,3),np.uint8)), {**defaults()['missions']['crosswalk'], **changes})


def test_threshold_and_hsv_control_change_classification():
    frame = np.full((60,80,3), 150, np.uint8)
    params = defaults()['missions']['crosswalk']
    assert render_preview(encoded(frame), params)['detected']
    assert not render_preview(encoded(frame), {**params, 'white_v_min': 200})['detected']


def test_latest_bag_frames_are_read_only_and_preserve_rgb_pixels(tmp_path):
    bag = tmp_path / 'recording'; bag.mkdir(); (bag/'metadata.yaml').write_text('')
    path = bag/'data.db3'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        db.execute("INSERT INTO topics VALUES(1,'/camera/color/image_raw','sensor_msgs/msg/Image')")
        for index, rgb in enumerate([(255,0,0),(0,255,0)]):
            msg = Image(height=10,width=10,encoding='rgb8',step=32,
                        data=b''.join(bytes(rgb)*10+b'XX' for _ in range(10)))
            db.execute('INSERT INTO messages VALUES(?,?,?,?)', (index+1,1,1_000_000_000+index*100_000_000,serialize_message(msg)))
    before = path.read_bytes()
    store = BagImages(tmp_path); meta = store.latest()
    assert meta['count'] == 2 and meta['times'] == [0., .1]
    first, one = store.frame(0, meta['name']); second, two = store.frame(1, meta['name'])
    assert cv2.imdecode(np.frombuffer(first,np.uint8),1)[0,0].tolist() == [0,0,255]
    assert cv2.imdecode(np.frombuffer(second,np.uint8),1)[0,0].tolist() == [0,255,0]
    assert two['bag_time_sec'] == .1 and one['frame_index'] == 0
    with pytest.raises(ValueError): store.frame(2,meta['name'])
    with pytest.raises(ValueError): store.frame(0,'different-bag')
    assert path.read_bytes() == before

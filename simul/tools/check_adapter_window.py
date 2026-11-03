#!/usr/bin/env python3
"""Render actual Qt widgets and check toggles without an X server or simulator."""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'captures/adapter-check')
    options = parser.parse_args()
    import rclpy
    from limo_parking_sim.adapter_window import AdapterWindow
    from limo_parking_sim.config import load_config
    from limo_parking_sim.mission_adapter import create_adapter

    options.output.mkdir(parents=True, exist_ok=True)
    # Keep this verification separate from the normal simulator domain.
    rclpy.init(domain_id=179)
    node = create_adapter(load_config())
    window = None
    captures = []
    try:
        window = AdapterWindow(node)
        window.timer.stop()
        window.root.show()
        window.app.processEvents()
        assert window.app.font().exactMatch() or window.root.fontMetrics().inFont('주')

        def capture(name, size=(860, 900), all_rows=False, reveal=None):
            window.root.resize(*size)
            window.update_status()
            window.app.processEvents()
            if reveal is not None:
                for _ in range(3):
                    window.app.processEvents()
                    window.scroll.ensureWidgetVisible(window.rows[reveal]['card'], 0, 0)
                card = window.rows[reveal]['card']
                pos = card.mapTo(window.scroll.viewport(), window.QtCore.QPoint(0, 0))
                assert window.scroll.viewport().rect().contains(window.QtCore.QRect(pos, card.size()))
            root_rect = window.root.rect()
            for widget in (window.reselect_button, window.custom_topic, window.value_label, window.active_label):
                top_left = widget.mapTo(window.root, window.QtCore.QPoint(0, 0))
                assert root_rect.contains(window.QtCore.QRect(top_left, widget.size())), widget.objectName()
            for row in window.rows.values():
                for key in ('toggle', 'status', 'name', 'topic_label'):
                    widget = row[key]
                    pos = widget.mapTo(row['card'], window.QtCore.QPoint(0, 0))
                    assert row['card'].rect().contains(window.QtCore.QRect(pos, widget.size())), key
            if all_rows:
                assert window.scroll.verticalScrollBar().maximum() == 0, 'Default window clips a command row'
            path = options.output / f'{name}.png'
            assert window.root.grab().save(str(path))
            captures.append({'image': path.name, 'size': list(size), 'enabled': sorted(node.router.relays),
                             'selected': node.router.selected(node.sim_time(), time.monotonic())})

        def feed(topic, speed=.2):
            now = node.sim_time()
            assert node.router.receive(topic, speed, .1, now, now, time.monotonic())

        parking = '/mission/parking/command_03'
        cone = '/mission/cone/command_04'
        lane = '/mission/lane'
        capture('01-all-off', all_rows=True)
        for topic in (lane, cone, parking):
            window.rows[topic]['toggle'].click()
            assert topic in node.router.relays
            feed(topic)
        capture('02-priority-parking', all_rows=True)
        assert node.router.selected(node.sim_time(), time.monotonic()) == parking
        window.rows[parking]['toggle'].click()
        feed(cone)
        feed(lane)
        capture('03-parking-off-cone-active', all_rows=True)
        assert node.router.selected(node.sim_time(), time.monotonic()) == cone
        capture('04-minimum-window', (720, 620))
        window.reselect_button.click()
        assert not node.router.relays and node.publisher is None
        capture('05-topic-reselection', all_rows=True)
        # Invalid input leaves the window usable; a long custom path can wrap.
        window.custom_topic.setText('invalid topic')
        window.add_custom()
        assert len(window.rows) == 7
        custom = '/mission/custom_controller_with_a_long_name/command_09'
        window.custom_topic.setText(custom)
        window.add_custom()
        assert custom in window.rows
        window.rows[custom]['toggle'].click()
        feed(custom)
        capture('06-custom-topic', (720, 620), reveal=custom)
        window.reselect_button.click()
        window.QtCore.QTimer.singleShot(30, window.close)
        window.run()
        assert window.closed
        report = {'passed': True, 'renderer': 'actual PyQt5 widgets / offscreen',
                  'checks': ['Korean font', 'widget bounds', 'all default rows visible',
                             'toggle ON/OFF', 'priority fallback', 'topic reselection',
                             'minimum window size', 'invalid and long custom topics', 'Qt event loop and close'],
                  'captures': captures}
        (options.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        if window is not None:
            window.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

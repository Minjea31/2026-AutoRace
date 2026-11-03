"""Qt mission toggles; the same widgets support desktop and offscreen rendering."""
import time

from .mission_adapter import MISSION_TOPICS, PRIORITIES
from .qt_support import load_qt


STYLE = '''
QWidget { color: #18253b; background: #f3f6fb; }
QLabel { background: transparent; }
QLabel#title { font-size: 23px; font-weight: 700; }
QLabel#muted { color: #64748b; }
QFrame#card { background: white; border: 1px solid #dce3ee; border-radius: 10px; }
QFrame#card[state="active"] { background: #eff8f4; border: 1px solid #42a77d; }
QLabel#badge { color: #52627a; background: #edf2f8; border-radius: 7px; font-weight: 700; }
QLabel#mission { font-size: 14px; font-weight: 600; }
QLabel#topic { color: #64748b; font-family: "DejaVu Sans Mono"; font-size: 11px; }
QLabel#rowStatus { color: #64748b; }
QLabel#rowStatus[state="active"] { color: #18774f; font-weight: 700; }
QPushButton { background: white; border: 1px solid #c9d4e4; border-radius: 7px; padding: 7px 13px; }
QPushButton:hover { border-color: #6f97c9; background: #edf3fd; }
QPushButton#toggle { color: #64748b; background: #eef2f7; font-weight: 700; }
QPushButton#toggle:checked { color: white; background: #2463ce; border-color: #2463ce; }
QPushButton#reselect { color: #2463ce; font-weight: 600; }
QLineEdit { background: white; border: 1px solid #c9d4e4; border-radius: 7px; padding: 7px 10px; }
QScrollArea { border: none; background: transparent; }
QFrame#output { background: #e7eef9; border-radius: 10px; }
QLabel#outputValue { font-size: 16px; font-weight: 700; }
'''


class AdapterWindow:
    def __init__(self, node):
        from rclpy.executors import SingleThreadedExecutor

        self.QtCore, self.QtGui, self.QtWidgets = load_qt()
        q = self.QtWidgets
        self.app = q.QApplication.instance() or q.QApplication([])
        self.app.setFont(self.QtGui.QFont('Noto Sans CJK KR', 10))
        self.node = node
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.closed = False
        self.root = q.QWidget()
        self.root.setWindowTitle('AutoRace · 미션 command 어댑터')
        self.root.resize(860, 900)
        self.root.setMinimumSize(720, 620)
        self.root.setStyleSheet(STYLE)
        self.root.closeEvent = self._close_event
        layout = q.QVBoxLayout(self.root)
        layout.setContentsMargins(24, 22, 24, 22)
        layout.setSpacing(12)
        title = q.QLabel('미션 command 연결')
        title.setObjectName('title')
        layout.addWidget(title)
        subtitle = q.QLabel(f'ROS domain {node.context.get_domain_id()}   ·   출력 {node.output_topic}   ·   시뮬레이션 시간')
        subtitle.setObjectName('muted')
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)
        help_text = q.QLabel('토글을 켜면 바로 연결됩니다. 여러 입력이 유효하면 01 → 02 → 03 → 04 → 05 → 08 순으로 전달하고, 차선은 기본 입력으로 사용합니다.')
        help_text.setObjectName('muted')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)

        self.scroll = q.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarAlwaysOff)
        self.list_widget = q.QWidget()
        self.rows_layout = q.QVBoxLayout(self.list_widget)
        self.rows_layout.setContentsMargins(0, 0, 6, 0)
        self.rows_layout.setSpacing(8)
        self.rows = {}
        for label, topic in MISSION_TOPICS:
            self.add_row(label, topic)
        for topic in node.router.relays:
            if topic not in self.rows:
                self.add_row('사용자 지정 입력', topic)
        self.rows_layout.addStretch()
        self.scroll.setWidget(self.list_widget)
        layout.addWidget(self.scroll, 1)

        custom = q.QHBoxLayout()
        self.custom_topic = q.QLineEdit()
        self.custom_topic.setPlaceholderText('사용자 지정 토픽 /mission/...')
        self.custom_topic.setAccessibleName('사용자 지정 입력 토픽')
        custom.addWidget(self.custom_topic, 1)
        add_button = q.QPushButton('입력 추가')
        add_button.clicked.connect(self.add_custom)
        self.custom_topic.returnPressed.connect(self.add_custom)
        custom.addWidget(add_button)
        layout.addLayout(custom)

        output = q.QFrame()
        output.setObjectName('output')
        output_layout = q.QVBoxLayout(output)
        output_layout.setContentsMargins(16, 12, 16, 12)
        self.active_label = q.QLabel()
        self.active_label.setWordWrap(True)
        self.value_label = q.QLabel()
        self.value_label.setObjectName('outputValue')
        output_layout.addWidget(self.active_label)
        output_layout.addWidget(self.value_label)
        layout.addWidget(output)
        footer = q.QHBoxLayout()
        self.message = q.QLabel('연결할 command의 토글을 켜세요.')
        self.message.setObjectName('muted')
        self.message.setWordWrap(True)
        footer.addWidget(self.message, 1)
        self.reselect_button = q.QPushButton('토픽 재선택')
        self.reselect_button.setObjectName('reselect')
        self.reselect_button.clicked.connect(self.reselect)
        footer.addWidget(self.reselect_button)
        layout.addLayout(footer)
        note = q.QLabel('입력 만료 0.25초 · 후진은 주차만 허용 · 미션 노드는 별도 실행 · 키보드 제어(K)는 꺼 주세요.')
        note.setObjectName('muted')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.timer = self.QtCore.QTimer(self.root)
        self.timer.timeout.connect(self.poll)
        self.timer.start(20)
        self.update_status()

    def add_row(self, label, topic):
        q = self.QtWidgets
        card = q.QFrame()
        card.setObjectName('card')
        card.setMinimumHeight(66)
        row = q.QHBoxLayout(card)
        row.setContentsMargins(12, 10, 12, 10)
        row.setSpacing(12)
        priority = PRIORITIES.get(topic, 100)
        badge = q.QLabel('기본' if priority == 1000 else (f'{priority:02}' if priority < 100 else '+'))
        badge.setObjectName('badge')
        badge.setAlignment(self.QtCore.Qt.AlignCenter)
        badge.setFixedSize(40, 34)
        row.addWidget(badge)
        text = q.QVBoxLayout()
        text.setSpacing(2)
        name = q.QLabel(label)
        name.setObjectName('mission')
        topic_label = q.QLabel(topic)
        topic_label.setObjectName('topic')
        topic_label.setToolTip(topic)
        topic_label.setWordWrap(True)
        text.addWidget(name)
        text.addWidget(topic_label)
        row.addLayout(text, 1)
        status = q.QLabel('연결 안 됨')
        status.setObjectName('rowStatus')
        status.setMinimumWidth(75)
        status.setAlignment(self.QtCore.Qt.AlignRight | self.QtCore.Qt.AlignVCenter)
        row.addWidget(status)
        toggle = q.QPushButton('OFF')
        toggle.setObjectName('toggle')
        toggle.setCheckable(True)
        toggle.setFixedSize(72, 34)
        toggle.setAccessibleName(f'{label} 입력 토글')
        toggle.setChecked(topic in self.node.router.relays)
        toggle.toggled.connect(lambda enabled, topic=topic: self.toggle_topic(topic, enabled))
        row.addWidget(toggle)
        # Custom rows are inserted ahead of the trailing stretch.
        self.rows_layout.insertWidget(len(self.rows), card)
        self.rows[topic] = {'card': card, 'toggle': toggle, 'status': status,
                            'topic_label': topic_label, 'name': name}

    def toggle_topic(self, topic, enabled):
        try:
            self.node.enable_topic(topic, enabled)
        except (ValueError, RuntimeError) as exc:
            self.message.setText(str(exc))
        else:
            self.message.setText(f'{"ON" if enabled else "OFF"} · {topic}')
        self.update_status()

    def add_custom(self):
        from rclpy.exceptions import InvalidTopicNameException
        from rclpy.validate_full_topic_name import validate_full_topic_name

        topic = self.custom_topic.text().strip()
        try:
            validate_full_topic_name(topic)
            if topic == self.node.output_topic:
                raise ValueError('입력 토픽과 출력 토픽은 달라야 합니다.')
        except (InvalidTopicNameException, ValueError) as exc:
            self.message.setText(str(exc))
            return
        if topic not in self.rows:
            self.add_row('사용자 지정 입력', topic)
        self.QtCore.QTimer.singleShot(0, lambda: self.scroll.ensureWidgetVisible(self.rows[topic]['card']))
        self.custom_topic.clear()
        self.message.setText('입력을 추가했습니다. 토글을 켜면 연결됩니다. 사용자 지정 입력은 08 다음, 차선보다 먼저 선택합니다.')

    def reselect(self):
        self.node.disconnect()
        self.message.setText('모든 입력을 해제했습니다. 연결할 토픽의 토글을 다시 켜세요.')
        self.update_status()

    def update_status(self):
        router = self.node.router
        now, wall = self.node.sim_time(), time.monotonic()
        active = router.selected(now, wall)
        for topic, row in self.rows.items():
            relay = router.relays.get(topic)
            enabled = relay is not None
            toggle = row['toggle']
            toggle.blockSignals(True)
            toggle.setChecked(enabled)
            toggle.setText('ON' if enabled else 'OFF')
            toggle.blockSignals(False)
            state = 'active' if active == topic else 'idle'
            for widget in (row['card'], row['status']):
                if widget.property('state') != state:
                    widget.setProperty('state', state)
                    widget.style().unpolish(widget)
                    widget.style().polish(widget)
            status = ('전달 중' if active == topic else
                      '대기 · 유효' if enabled and relay.fresh(now, wall) else
                      '수신 대기' if enabled else '연결 안 됨')
            row['status'].setText(status)
            row['status'].setToolTip(f'유효 입력 {relay.count if enabled else 0}회')
        self.active_label.setText(f'현재 전달: {active or "없음 · 정지"}')
        speed, yaw = router.output(now, wall)
        self.value_label.setText(f'{speed:.3f} m/s    /    {yaw:.3f} rad/s')

    def poll(self):
        from rclpy.executors import ExternalShutdownException

        if self.closed:
            return
        if not self.node.context.ok():
            self.close()
            return
        try:
            for _ in range(8):
                self.executor.spin_once(timeout_sec=0)
        except ExternalShutdownException:
            self.close()
            return
        self.update_status()

    def _close_event(self, event):
        if not self.closed:
            self.closed = True
            self.timer.stop()
            if self.node.context.ok():
                self.node.disconnect()
            self.executor.shutdown()
        event.accept()

    def close(self):
        self.root.close()

    def run(self):
        self.root.show()
        self.app.exec()

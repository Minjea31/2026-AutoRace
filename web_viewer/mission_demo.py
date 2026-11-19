"""Static preview of the three monitor panels without ROS."""


def load_demo(state):
    state.set_status('owner', 'objavoid', True)
    state.set_status('slow', False, True)
    state.set_status('objavoid', True, True)
    state.set_status('parking', False, True)
    return {
        'scan': [[0.8 + i * 0.06, 0.55] for i in range(40)],
        'markers': [
            {'ns': 'candidates', 'type': 5, 'points': [[0, 0], [1.2, 0.3], [1.2, 0.3], [2.2, 0.15]],
             'scale': 0.02, 'color': [0.4, 0.85, 1, 1]},
            {'ns': 'path', 'type': 4, 'points': [[0, 0], [0.8, -0.1], [1.5, -0.28], [2.5, 0.06]],
             'scale': 0.035, 'color': [1, 0.2, 0.6, 1]},
        ],
    }

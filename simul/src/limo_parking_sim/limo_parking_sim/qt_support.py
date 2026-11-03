"""Load Conda Qt, or the system PyQt5 supplied by a ROS desktop install."""
import importlib.machinery
import importlib.util
import sys


def load_qt():
    try:
        import PyQt5
    except ModuleNotFoundError:
        spec = importlib.machinery.PathFinder.find_spec('PyQt5', ['/usr/lib/python3/dist-packages'])
        if spec is None:
            raise RuntimeError('선택 창에 PyQt5가 필요합니다. conda install -n autorace-simul -c conda-forge pyqt=5.15')
        package = importlib.util.module_from_spec(spec)
        sys.modules['PyQt5'] = package
        spec.loader.exec_module(package)
    from PyQt5 import QtCore, QtGui, QtWidgets
    return QtCore, QtGui, QtWidgets

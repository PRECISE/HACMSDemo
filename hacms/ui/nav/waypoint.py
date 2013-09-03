# QT modules
from PyQt4.QtGui import *
from PyQt4.QtMobility.QtLocation import *

class Waypoint(QGeoMapPixmapObject):
    def __init__(coordinate, offset):
        QGeoMapPixmapObject.__init__(coordinate, offset)
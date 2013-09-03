# QT modules
from PyQt4.QtGui import *
from PyQt4.QtMobility.QtLocation import *

class NavScene(QGraphicsScene):
    def __init__(self, parent=None): 
        QGraphicsScene.__init__(self, parent)
        
        self.serviceProvider = QGeoServiceProvider("osm")
        self.mappingManager = self.serviceProvider.mappingManager()
        self.geoMap = QGraphicsGeoMap(self.mappingManager)
        
        self.waypoints = []
        
        # http://doc.qt.digia.com/qtmobility-1.2/qgeomappolylineobject.html
        self.waypointPaths = []
        
        self.addWidget(self.geoMap)

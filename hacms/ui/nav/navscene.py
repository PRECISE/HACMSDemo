# QT modules
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4.QtMobility.QtLocation import *

import ui.images_rc
from waypoint import Waypoint

#TODO: 

class NavScene(QGraphicsScene):
    def __init__(self, parent=None):
        super(NavScene, self).__init__(parent)

#         self.serviceProvider = QGeoServiceProvider("nokia")
#         self.mappingManager = self.serviceProvider.mappingManager()
#         self.geoMap = QGraphicsGeoMap(self.mappingManager)

        # http://doc.qt.digia.com/qtmobility-1.2/qgeomappolylineobject.html
        self.waypointPath = None

        #self.addWidget(self.geoMap)

        #TODO: Plot GPS coordinates as line on X-Y graph
        
        self.addPixmap(QPixmap(":/images/navMap1.png"))
        self.waypoints = []
        self.waypointStringList = QStringList()
        self.listView = None
        self.upperLeftCoordinate = (39.954221, -75.19183)
        self.upperRightCoordinate = (39.954233, -75.189858)
        self.lowerLeftCoordinate = (39.953267, -75191862)
        self.lowerRightCoordinate = (39.953266, -75.189896)
        self.coordWidth = abs(self.upperLeftCoordinate[1] - self.upperRightCoordinate[1])
        self.coordHeight = abs(self.upperLeftCoordinate[0] - self.lowerLeftCoordinate[0])
        self.pixelDegreeRatio = self.width() / self.coordWidth
        
        self.waypointPath = []

    def setListView(self, view):
        self.listView = view

    def addWaypoint(self, waypoint):
        # Set source and destination values on waypoints, if some already exist in the list
        if(len(self.waypoints) > 0):
            self.waypoints[-1].setDestination(waypoint)
            waypoint.setSource(self.waypoints[-1])
        
        self.waypoints.append(waypoint)
        self.waypointStringList.append(waypoint.toString())
        sceneWp = self.addPixmap(waypoint.pixmap)
        sceneWp.setOffset(-7, -24)
        sceneWp.setPos(self.coordToPoint(waypoint.coordinate))
        sceneWp.setZValue(2)
        waypoint.setGraphicsPixmapItem(sceneWp)
        waypoint.setScene(self)
        
        self.listView.setModel(QStringListModel(self.waypointStringList))
        self.drawRoute()
        
    def clearWaypoints(self):
        for w in self.waypoints:
            w.clearGraphicsItems()
        for l in self.waypointPath:
            self.removeItem(l)
        self.waypoints = []
        self.waypointStringList = QStringList()
        self.listView.setModel(QStringListModel(self.waypointStringList))
        
    def drawRoute(self):
        pen = QPen()
        pen.setColor(QColor("gold"))
        pen.setWidth(4)
        for w in self.waypoints:
            if w.destination is not None:
                line = self.addLine(w.graphicsPixmapItem.x(), w.graphicsPixmapItem.y(), w.destination.graphicsPixmapItem.x(), w.destination.graphicsPixmapItem.y(), pen)
                line.setZValue(1) # Move line to be drawn under waypoints
                w.setDestinationLine(line)
                
                self.waypointPath.append(line)

    def coordToPoint(self, coordinate):
        x = abs(self.upperLeftCoordinate[1] - coordinate.longitude()) * self.pixelDegreeRatio
        y = abs(self.upperLeftCoordinate[0] - coordinate.latitude()) * self.pixelDegreeRatio
        return QPointF(x,y)
        
    def pointToCoord(self, point):
        lat = self.upperLeftCoordinate[0] - (point.y() * (1 / self.pixelDegreeRatio))
        long = self.upperLeftCoordinate[1] - (point.x() * (1 / self.pixelDegreeRatio))
        return QGeoCoordinate(lat, long)
        
    def mousePressEvent(self, event):
        coord = self.pointToCoord(event.buttonDownScenePos(Qt.LeftButton))
        self.addWaypoint(Waypoint(coord.latitude(), coord.longitude()))

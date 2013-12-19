# QT modules
from PyQt4.QtGui import *
from PyQt4.QtMobility.QtLocation import *
import ui.images_rc

#class Waypoint(QGeoMapPixmapObject):
class Waypoint():
    def __init__(self, lat, long, source = None, destination = None):
        #QGeoMapPixmapObject.__init__(coordinate, offset, QPixmap("images/map_pin_stroke_15x24.png"))    
        self.coordinate = QGeoCoordinate(lat, long)
        self.source = source
        self.sourceLine = None
        self.destination = destination
        self.destinationLine = None
        self.pixmap = QPixmap(":/images/map_pin_stroke_15x24.png")
        
    def setGraphicsPixmapItem(self, item):
        self.graphicsPixmapItem = item
        
    def setScene(self, scene):
        self.scene = scene
        
    def setSource(self, waypoint):
        self.source = waypoint
    
    def setDestination(self, waypoint):
        self.destination = waypoint
        
    def setSourceLine(self, line):
        self.sourceLine = line
    
    def setDestinationLine(self, line):
        self.destinationLine = line
        
    def clearGraphicsItems(self):
        if self.sourceLine is not None:
            self.scene.removeItem(self.sourceLine)
        if self.destinationLine is not None:
            self.scene.removeItem(self.destinationLine)
        self.scene.removeItem(self.graphicsPixmapItem)
        
    def toString(self):
        return self.coordinate.toString(QGeoCoordinate.Degrees)

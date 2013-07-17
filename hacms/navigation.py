# From http://old.nabble.com/Simple-PyQt4-Application-with-Mapnik-without-PyQGIS-(QGIS-API)-td29243400.html

import mapnik2 as mapnik
from PyQt4.QtGui import *
from PyQt4.QtCore import *

class MapnikScene(QGraphicsScene):

   def __init__(self, parent=None): 
       QGraphicsScene.__init__(self, parent)
       
       self.pixmap = QPixmap(256, 256)
       self.map = mapnik.Map(256, 256)
       self.startDragPos = QPoint() 
       self.endDragPos   = QPoint() 
       self.drag         = False 
       self.scale        = False 
       self.timer        = QTimer() 

       self.timer.timeout.connect(self.updateMap) 

       self.total_scale = 1.0
       
       self.addPixmap(self.pixmap)

   def load_map(self, xml): 
       self.map = mapnik.Map(256, 256) 
       mapnik.load_map(self.map, xml) 
       self.map.resize(self.width(), self.height()) 
       self.zoom_all() 

   def close_map(self): 
       self.map = mapnik.Map(256, 256) 
       self.updateMap() 

   def updateMap(self): 
       self.timer.stop() 
       self.total_scale = 1.0 
       self.scale       = False 

       im = mapnik.Image(self.map.width, self.map.height) 
       mapnik.render(self.map, im) 
       #self.qim = QImage(im.tostring(), self.map.width, self.map.height, QImage.Format_ARGB32).rgbSwapped() 
       self.pixmap = QPixmap.fromImage(QImage.loadFromData(QByteArray(im.tostring('png'))))
       self.update() 

   def paintEvent(self, event): 
       painter = QPainter(self) 

       if self.drag: 
           painter.drawImage(self.endDragPos - self.startDragPos, self.qim) 
       elif self.scale: 
           qw = self.qim.width() 
           qh = self.qim.height() 
           newWidth = int(qw * self.total_scale) 
           newHeight = int(qh * self.total_scale) 
           newX = (qw - newWidth) / 2 
           newY = (qh - newHeight) / 2 
           painter.save() 
           painter.translate(newX, newY) 
           painter.scale(self.total_scale, self.total_scale) 
           exposed = painter.matrix().inverted()[0].mapRect(self.rect()).adjusted(-1, -1, 1, 1) 
           painter.drawImage(exposed, self.qim, exposed) 
           painter.restore() 
       else: 
           painter.drawImage(0, 0, self.qim) 

       painter.setPen(QColor(0, 0, 0, 100)) 
       painter.setBrush(QColor(0, 0, 0, 100)) 
       painter.drawRect(0, 0, 256, 26) 
       painter.setPen(QColor(0, 255 , 0)) 
       painter.drawText(10, 19, 'Scale Denominator: ' + str(self.map.scale_denominator())) 

   def zoom_all(self): 
       self.map.zoom_all() 
       self.updateMap() 

   def resizeEvent(self, event): 
       self.map.resize(event.size().width(), event.size().height()) 
       self.updateMap() 

   def wheelEvent(self, event): 
       self.scale = True 
       scale_factor = 1.0 - event.delta() / (360.0 * 8.0) * 4 
       self.map.zoom(scale_factor) 
       self.total_scale *= 1 / scale_factor 
       self.update() 
       self.timer.start(400) 

   def mousePressEvent(self, event): 
       if event.button() == Qt.LeftButton: 
           self.startDragPos = event.pos() 
           self.drag         = True 

   def mouseMoveEvent(self, event): 
       if self.drag: 
           self.endDragPos = event.pos() 
           self.update() 

   def mouseReleaseEvent(self, event): 
       if event.button() == Qt.LeftButton: 
           self.drag = False 
           self.endDragPos = event.pos() 

           cx = int(0.5 * self.map.width) 
           cy = int(0.5 * self.map.height) 
           dpos = self.endDragPos - self.startDragPos 
           self.map.pan(cx - dpos.x() ,cy - dpos.y()) 
           self.updateMap()

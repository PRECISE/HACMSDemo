from PyQt4.QtGui import *
from navscene import NavScene

class NavView(QGraphicsView):
    def __init__(self, parent=None):
        self.scene = NavScene(parent)
        super(NavView, self).__init__(self.scene, parent)
        self.show()
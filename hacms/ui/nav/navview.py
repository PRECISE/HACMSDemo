from PyQt4.QtGui import *
from navscene import NavScene

class NavView(QGraphicsView):
    def __init__(self, parent=None):
        scene = NavScene()
        super(NavView, self).__init__(scene, parent)
        self.show()

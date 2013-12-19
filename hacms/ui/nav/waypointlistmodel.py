# QT modules
from PyQt4.QtCore import *

class WaypointListModel(QAbstractListModel):
    def __init__(self, list = None):
        if list:
            self.data = list

    def data(self, modelIndex):
        return self.data[modelIndex.row()].toString()

    def rowCount(self):
        return len(self.data)

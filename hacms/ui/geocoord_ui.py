# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'geocoord.ui'
#
# Created: Mon Oct  7 12:38:19 2013
#      by: PyQt4 UI code generator 4.10.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_GeoCoordinateDialog(object):
    def setupUi(self, GeoCoordinateDialog):
        GeoCoordinateDialog.setObjectName(_fromUtf8("GeoCoordinateDialog"))
        GeoCoordinateDialog.setWindowModality(QtCore.Qt.ApplicationModal)
        GeoCoordinateDialog.resize(265, 101)
        GeoCoordinateDialog.setModal(True)
        self.buttonBox = QtGui.QDialogButtonBox(GeoCoordinateDialog)
        self.buttonBox.setGeometry(QtCore.QRect(10, 60, 241, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(True)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.latitudeLineEdit = QtGui.QLineEdit(GeoCoordinateDialog)
        self.latitudeLineEdit.setGeometry(QtCore.QRect(10, 30, 113, 22))
        self.latitudeLineEdit.setObjectName(_fromUtf8("latitudeLineEdit"))
        self.longitudeLineEdit = QtGui.QLineEdit(GeoCoordinateDialog)
        self.longitudeLineEdit.setGeometry(QtCore.QRect(140, 30, 113, 22))
        self.longitudeLineEdit.setObjectName(_fromUtf8("longitudeLineEdit"))
        self.latitudeLabel = QtGui.QLabel(GeoCoordinateDialog)
        self.latitudeLabel.setGeometry(QtCore.QRect(10, 10, 91, 16))
        self.latitudeLabel.setObjectName(_fromUtf8("latitudeLabel"))
        self.longitudeLabel = QtGui.QLabel(GeoCoordinateDialog)
        self.longitudeLabel.setGeometry(QtCore.QRect(140, 10, 101, 16))
        self.longitudeLabel.setObjectName(_fromUtf8("longitudeLabel"))
        self.latitudeLabel.setBuddy(self.latitudeLineEdit)
        self.longitudeLabel.setBuddy(self.longitudeLineEdit)

        self.retranslateUi(GeoCoordinateDialog)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("accepted()")), GeoCoordinateDialog.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("rejected()")), GeoCoordinateDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(GeoCoordinateDialog)
        GeoCoordinateDialog.setTabOrder(self.latitudeLineEdit, self.longitudeLineEdit)
        GeoCoordinateDialog.setTabOrder(self.longitudeLineEdit, self.buttonBox)

    def retranslateUi(self, GeoCoordinateDialog):
        GeoCoordinateDialog.setWindowTitle(_translate("GeoCoordinateDialog", "GeoCoordinate Entry", None))
        self.latitudeLabel.setText(_translate("GeoCoordinateDialog", "Latitude", None))
        self.longitudeLabel.setText(_translate("GeoCoordinateDialog", "Longitude", None))


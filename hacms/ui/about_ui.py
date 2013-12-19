# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'about.ui'
#
# Created: Tue Oct  1 15:16:57 2013
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

class Ui_AboutDialog(object):
    def setupUi(self, AboutDialog):
        AboutDialog.setObjectName(_fromUtf8("AboutDialog"))
        AboutDialog.setWindowModality(QtCore.Qt.ApplicationModal)
        AboutDialog.resize(464, 240)
        AboutDialog.setMinimumSize(QtCore.QSize(464, 240))
        AboutDialog.setMaximumSize(QtCore.QSize(464, 240))
        AboutDialog.setModal(True)
        self.aboutText = QtGui.QLabel(AboutDialog)
        self.aboutText.setGeometry(QtCore.QRect(0, 110, 461, 131))
        self.aboutText.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.aboutText.setWordWrap(True)
        self.aboutText.setMargin(15)
        self.aboutText.setOpenExternalLinks(True)
        self.aboutText.setObjectName(_fromUtf8("aboutText"))
        self.pennLogo = QtGui.QLabel(AboutDialog)
        self.pennLogo.setGeometry(QtCore.QRect(-1, 12, 461, 91))
        self.pennLogo.setFrameShape(QtGui.QFrame.NoFrame)
        self.pennLogo.setFrameShadow(QtGui.QFrame.Plain)
        self.pennLogo.setText(_fromUtf8(""))
        self.pennLogo.setPixmap(QtGui.QPixmap(_fromUtf8(":/images/seas_logo.png")))
        self.pennLogo.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignHCenter)
        self.pennLogo.setObjectName(_fromUtf8("pennLogo"))

        self.retranslateUi(AboutDialog)
        QtCore.QMetaObject.connectSlotsByName(AboutDialog)

    def retranslateUi(self, AboutDialog):
        AboutDialog.setWindowTitle(_translate("AboutDialog", "About HACMS Demo", None))
        self.aboutText.setText(_translate("AboutDialog", "<html><head/><body><p>The <span style=\" font-weight:600;\">HACMS Demo</span> application allows for control of the LandShark robot while displaying live ROS telemetry data. </p><p>Developed by the <a href=\"http://precise.seas.upenn.edu\"><span style=\" text-decoration: underline; color:#0000ff;\">PRECISE Center</span></a> at <a href=\"http://www.seas.upenn.edu\"><span style=\" text-decoration: underline; color:#0000ff;\">University of Pennsylvania</span></a>.</p><p>Authors: <a href=\"http://www.seas.upenn.edu/~pgeb\"><span style=\" text-decoration: underline; color:#0000ff;\">Peter Gebhard</span></a>, <a href=\"http://www.seas.upenn.edu/~nicbezzo\"><span style=\" text-decoration: underline; color:#0000ff;\">Nicola Bezzo</span></a></p></body></html>", None))

import images_rc

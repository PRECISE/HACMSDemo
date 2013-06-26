# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Wed Jun 26 13:15:30 2013
#      by: pyside-uic 0.2.13 running on PySide 1.1.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(481, 380)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.startRobotButton = QtGui.QPushButton(self.centralWidget)
        self.startRobotButton.setGeometry(QtCore.QRect(20, 20, 201, 41))
        self.startRobotButton.setObjectName("startRobotButton")
        self.startRCButton = QtGui.QPushButton(self.centralWidget)
        self.startRCButton.setGeometry(QtCore.QRect(20, 60, 201, 41))
        self.startRCButton.setObjectName("startRCButton")
        self.attackButton = QtGui.QPushButton(self.centralWidget)
        self.attackButton.setGeometry(QtCore.QRect(240, 20, 171, 81))
        self.attackButton.setCheckable(True)
        self.attackButton.setObjectName("attackButton")
        self.actualSpeedLCD = QtGui.QLCDNumber(self.centralWidget)
        self.actualSpeedLCD.setGeometry(QtCore.QRect(40, 220, 171, 71))
        self.actualSpeedLCD.setObjectName("actualSpeedLCD")
        self.expectedLabel = QtGui.QLabel(self.centralWidget)
        self.expectedLabel.setGeometry(QtCore.QRect(250, 130, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(36)
        self.expectedLabel.setFont(font)
        self.expectedLabel.setStyleSheet("background-color: green; color: white;")
        self.expectedLabel.setObjectName("expectedLabel")
        self.actualLabel = QtGui.QLabel(self.centralWidget)
        self.actualLabel.setGeometry(QtCore.QRect(250, 230, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(36)
        self.actualLabel.setFont(font)
        self.actualLabel.setStyleSheet("background-color: green; color: white;")
        self.actualLabel.setObjectName("actualLabel")
        self.desiredSpeedLabel = QtGui.QLabel(self.centralWidget)
        self.desiredSpeedLabel.setGeometry(QtCore.QRect(70, 120, 101, 16))
        self.desiredSpeedLabel.setObjectName("desiredSpeedLabel")
        self.desiredSpeedEdit = QtGui.QLineEdit(self.centralWidget)
        self.desiredSpeedEdit.setGeometry(QtCore.QRect(60, 150, 113, 21))
        self.desiredSpeedEdit.setObjectName("desiredSpeedEdit")
        self.actualSpeedLabel = QtGui.QLabel(self.centralWidget)
        self.actualSpeedLabel.setGeometry(QtCore.QRect(80, 200, 121, 16))
        self.actualSpeedLabel.setObjectName("actualSpeedLabel")
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar()
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 481, 22))
        self.menuBar.setObjectName("menuBar")
        self.menuFile = QtGui.QMenu(self.menuBar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)
        self.actionQuit = QtGui.QAction(MainWindow)
        self.actionQuit.setObjectName("actionQuit")
        self.menuFile.addAction(self.actionQuit)
        self.menuBar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "HACMS Demo", None, QtGui.QApplication.UnicodeUTF8))
        self.startRobotButton.setText(QtGui.QApplication.translate("MainWindow", "Start Robot", None, QtGui.QApplication.UnicodeUTF8))
        self.startRCButton.setText(QtGui.QApplication.translate("MainWindow", "Start Resilient Controller", None, QtGui.QApplication.UnicodeUTF8))
        self.attackButton.setText(QtGui.QApplication.translate("MainWindow", "Attack", None, QtGui.QApplication.UnicodeUTF8))
        self.expectedLabel.setText(QtGui.QApplication.translate("MainWindow", "Expected", None, QtGui.QApplication.UnicodeUTF8))
        self.actualLabel.setText(QtGui.QApplication.translate("MainWindow", "Actual", None, QtGui.QApplication.UnicodeUTF8))
        self.desiredSpeedLabel.setText(QtGui.QApplication.translate("MainWindow", "Desired Speed", None, QtGui.QApplication.UnicodeUTF8))
        self.desiredSpeedEdit.setText(QtGui.QApplication.translate("MainWindow", "3.0", None, QtGui.QApplication.UnicodeUTF8))
        self.actualSpeedLabel.setText(QtGui.QApplication.translate("MainWindow", "Actual Speed", None, QtGui.QApplication.UnicodeUTF8))
        self.menuFile.setTitle(QtGui.QApplication.translate("MainWindow", "File", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setText(QtGui.QApplication.translate("MainWindow", "Quit", None, QtGui.QApplication.UnicodeUTF8))


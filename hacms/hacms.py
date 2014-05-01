#!/usr/bin/env python

# Copyright (c) 2013, The Trustees of the University of Pennsylvania.
# Developed with the sponsorship of the Defense Advanced Research Projects
# Agency (DARPA).
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this data, including any software or models in source or binary
# form, as well as any drawings, specifications, and documentation
# (collectively "the Data"), to deal in the Data without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Data, and to
# permit persons to whom the Data is furnished to do so, subject to the
# following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Data.
#
# THE DATA IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS, SPONSORS, DEVELOPERS, CONTRIBUTORS, OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE DATA OR THE USE OR OTHER DEALINGS IN THE DATA.

# Authors: Peter Gebhard (pgeb@seas.upenn.edu), Nicola Bezzo (nicbezzo@seas.upenn.edu)

# Standard Python modules
import sys, string
from collections import deque

#ROSPy modules
import rospy
import actionlib
from std_msgs.msg import Int32, Float32, Float64
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
import roslib; roslib.load_manifest('landshark_msgs')
from landshark_msgs.msg import NavigateToWayPointsGoal, NavigateToWayPointsAction, NavigateToWayPointsFeedback, NavigateToWayPointsActionFeedback, NavigateToWayPointsResult, NavigateToWayPointsActionGoal
import threading

# QT modules
from PyQt4.QtGui import *
from PyQt4.QtCore import *

# PyQtGraph
import pyqtgraph as pg

# HACMS modules
from remote import Remote
import ui.images_rc
import ui.about_ui
from ui.geocoord import GeoCoordDialog
from ui.nav.waypoint import Waypoint
from ui.nav.navscene import NavScene

#TODO: Layout widgets so that the console and plots will resize with the window

class HACMSWindow(QMainWindow):
    def __init__(self):
        super(HACMSWindow, self).__init__()

    def init_window(self):
        self.ui.setupUi(self)
        self.remote = Remote(self.ui.console)
        self.init_data_structs()
        self.init_widgets()

    def init_data_structs(self):
        self.trimIncrement = 0.001
        self.windowSize = 300
        self.in_Base = deque(maxlen=self.windowSize)
        self.in_Ref = deque(maxlen=self.windowSize)
        self.out_Odom = deque(maxlen=self.windowSize)
        self.out_EncL = deque(maxlen=self.windowSize)
        self.out_EncR = deque(maxlen=self.windowSize)
        self.out_GPS = deque(maxlen=self.windowSize)
        self.resilientControllerRegions = []

        self.datastructs = [
            self.in_Base,
            self.in_Ref,
            self.out_Odom,
            self.out_EncL,
            self.out_EncR,
            self.out_GPS
        ]
        self.init_waypoints()

    def init_waypoints(self):
        self.waypointList = QStringList()
        self.waypointList.append("test")
        self.waypointModel = QStringListModel()
        self.ui.waypointListView.setModel(self.waypointModel)
        self.waypointModel.setStringList(self.waypointList)
        self.ui.navView.setScene(NavScene(self.ui.navView))
        self.ui.navView.scene().setListView(self.ui.waypointListView)
        self.ui.navView.scene().addWaypoint(Waypoint(39.953773, -75.191427))
        self.ui.navView.scene().addWaypoint(Waypoint(39.953679, -75.191304))
        self.ui.navView.scene().addWaypoint(Waypoint(39.953703, -75.191036))
        self.ui.navView.scene().addWaypoint(Waypoint(39.953769, -75.1908))

    def init_widgets(self):
        self.widgets = [
            self.ui.tabWidget,
            self.ui.ccButton,
            self.ui.rcButton,
            self.ui.attackButton,
            self.ui.attackComboBox,
            self.ui.gpsCheckBox,
            self.ui.enclCheckBox,
            self.ui.encrCheckBox,
            self.ui.saveButton,
            self.ui.desiredSpeedLabel,
            self.ui.desiredSpeedEdit,
            self.ui.setSpeedButton,
            self.ui.kpLabel,
            self.ui.kpEdit,
            self.ui.setKPButton,
            self.ui.kiLabel,
            self.ui.kiEdit,
            self.ui.setKIButton,
            self.ui.trimLabel,
            self.ui.trimValueLabel,
            self.ui.setTrimLeftButton,
            self.ui.setTrimRightButton,
            self.ui.actualSpeedLabel,
            self.ui.actualSpeedLCD,
            self.ui.estimatedSpeedLabel,
            self.ui.estimatedSpeedLCD,
            self.ui.outputPlot,
            self.ui.outputPlotLabel,
            self.ui.inputPlot,
            self.ui.inputPlotLabel,
            self.ui.rightPlot,
            self.ui.rightPlotLabel,
            self.ui.waypointLabel,
            self.ui.waypointListView,
            self.ui.addWaypointButton,
            self.ui.removeWaypointButton,
            self.ui.clearWaypointsButton,
            self.ui.uploadWaypointsButton
        ]
        self.init_signals()
        self.init_plots()

    def init_signals(self):
        self.ui.actionAbout.triggered.connect(self.about)
        self.ui.actionQuit.triggered.connect(self.fileQuit)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.attackComboBox.currentIndexChanged.connect(self.attackMode)
        self.ui.gpsCheckBox.toggled.connect(self.attackSensor)
        self.ui.enclCheckBox.toggled.connect(self.attackSensor)
        self.ui.encrCheckBox.toggled.connect(self.attackSensor)
        self.ui.saveButton.toggled.connect(self.saveData)
        self.ui.setSpeedButton.clicked.connect(self.setLandsharkSpeed)
        self.ui.setKPButton.clicked.connect(self.setKP)
        self.ui.setKIButton.clicked.connect(self.setKI)
        self.ui.setTrimLeftButton.clicked.connect(self.setTrimLeft)
        self.ui.setTrimRightButton.clicked.connect(self.setTrimRight)
        self.ui.addWaypointButton.clicked.connect(self.addWaypoint)
        self.ui.removeWaypointButton.clicked.connect(self.removeWaypoint)
        self.ui.clearWaypointsButton.clicked.connect(self.clearWaypoints)
        self.ui.uploadWaypointsButton.clicked.connect(self.uploadWaypoints)

        # Set Validator for parameter fields
        self.validator = QDoubleValidator()
        self.validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.desiredSpeedEdit.setValidator(self.validator)
        self.ui.kpEdit.setValidator(self.validator)
        self.ui.kiEdit.setValidator(self.validator)

    def init_plots(self):
        self.ui.inputPlot.disableAutoRange(pg.ViewBox.YAxis)
        self.ui.inputPlot.setYRange(0, 1.3, 0)
        self.ui.inputPlot.setBackground('w')
        self.ui.inputPlot.hideButtons()
        self.ui.inputPlot.showGrid(False, True)
        self.ui.inputPlot.setLabel('top', ' ')
        self.ui.inputPlot.setLabel('right', ' ')
        self.ui.inputPlot.setLabel('bottom', 'time')
        self.inputPlotBase = self.ui.inputPlot.plot(self.in_Base, name='CMD')
        self.inputPlotBase.setPen(pg.mkPen(width=3, color='r'))
        self.inputPlotRef = self.ui.inputPlot.plot(self.in_Ref, name='REF')
        self.inputPlotRef.setPen(pg.mkPen(width=3, color='c'))
        self.inputPlotTimer = QTimer()
        self.inputPlotTimer.timeout.connect(self.updateInputPlot)

        self.ui.outputPlot.setBackground('w')
        self.ui.outputPlot.hideButtons()
        self.ui.outputPlot.showGrid(False, True)
        self.ui.outputPlot.setLabel('top', ' ')
        self.ui.outputPlot.setLabel('right', ' ')
        self.ui.outputPlot.setLabel('bottom', 'time')
        self.outputPlotGPS = self.ui.outputPlot.plot(self.out_GPS, name='GPS')
        self.outputPlotGPS.setPen(pg.mkPen(width=3, color='m'))
        self.outputPlotLE = self.ui.outputPlot.plot(self.out_EncL, name='ENC LEFT')
        self.outputPlotLE.setPen(pg.mkPen(width=3, color='b'))
        self.outputPlotRE = self.ui.outputPlot.plot(self.out_EncR, name='ENC RIGHT')
        self.outputPlotRE.setPen(pg.mkPen(width=3, color='g'))
        self.outputPlotTimer = QTimer()
        self.outputPlotTimer.timeout.connect(self.updateOutputPlot)

        self.ui.rightPlot.disableAutoRange(pg.ViewBox.YAxis)
        self.ui.rightPlot.setYRange(0, 1.3, 0)
        self.ui.rightPlot.setBackground('w')
        self.ui.rightPlot.hideButtons()
        self.ui.rightPlot.showGrid(False, True)
        self.ui.rightPlot.setLabel('top', ' ')
        self.ui.rightPlot.setLabel('right', ' ')
        self.ui.rightPlot.setLabel('bottom', 'time')
        self.rightPlotOdom = self.ui.rightPlot.plot(self.out_Odom, name='SPEED')
        self.rightPlotOdom.setPen(pg.mkPen(width=3, color='b'))
        self.rightPlotRef = self.ui.rightPlot.plot(self.in_Ref, name='REF')
        self.rightPlotRef.setPen(pg.mkPen(width=3, color='c'))
        self.rightPlotTimer = QTimer()
        self.rightPlotTimer.timeout.connect(self.updateRightPlot)

        self.regionBrush = pg.mkBrush((93,220,131,50))

    def startPlotTimers(self):
        timerMsec = 500
        self.inputPlotTimer.start(timerMsec)
        self.outputPlotTimer.start(timerMsec)
        self.rightPlotTimer.start(timerMsec)

    def stopPlotTimers(self):
        self.inputPlotTimer.stop()
        self.outputPlotTimer.stop()
        self.rightPlotTimer.stop()
        
    def zeroData(self):
        for region in self.resilientControllerRegions:
            self.ui.rightPlot.removeItem(region)
        self.resilientControllerRegions = []
        for data in self.datastructs:
            data.clear()

    def about(self):
        about = QDialog()
        about.ui = ui.about_ui.Ui_AboutDialog()
        about.ui.setupUi(about)
        about.exec_()

    def fileQuit(self):
        if self.ui.landsharkButton.isChecked():
            self.stop_landshark_comm()
            self.remote.stopLandshark()
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def enableAllElements(self):
        for widget in self.widgets:
            widget.setEnabled(True)
        self.zeroData()

    def disableAllElements(self):
        self.cc(False)
        self.rc(False)
        self.attack(False)
        self.saveData(False)

        for widget in self.widgets:
            widget.setEnabled(False)

    def landshark(self, checked):
        if checked:
            res = self.remote.startLandshark()
            self.start_landshark_comm()
            self.enableAllElements()
            self.startPlotTimers()
        else:
            self.stopPlotTimers()
            self.disableAllElements()
            self.stop_landshark_comm()
            res = self.remote.stopLandshark()
        self.ui.landsharkButton.setChecked(res)

    def cc(self, checked):
        if checked:
            res = self.remote.startCC()
            self.zeroData()
        else:
            self.saveData(False)
            self.rc(False)
            self.attack(False)
            res = self.remote.stopCC()
        self.ui.ccButton.setChecked(res)

    def saveData(self, checked):
        if checked:
            res = self.remote.startSaveData()
        else:
            res = self.remote.stopSaveData()
        self.ui.saveButton.setChecked(res)

    def rc(self, checked):
        if checked:
            try:
                self.run_rc_pub.publish(Int32(1))
                currentRegion = pg.LinearRegionItem(values=[len(self.out_Odom), len(self.out_Odom)], brush=self.regionBrush, movable=False, bounds=[0,self.windowSize])
                self.resilientControllerRegions.append(currentRegion)
                self.ui.rightPlot.addItem(currentRegion)
            except:
                self.ui.rcButton.setChecked(False)
                return
        else:
            try:
                self.run_rc_pub.publish(Int32(0))
            except:
                self.ui.rcButton.setChecked(True)
                return

        # For when the button is set via direct method call, not by event call
        self.ui.rcButton.setChecked(checked)

    def attack(self, checked):
        if checked:
            try:
                self.run_attack_pub.publish(Int32(self.ui.attackComboBox.currentIndex()+1))
                self.sensor_attack_pub.publish(Int32(self.getAttackSensorValue()))
            except:
                self.ui.attackButton.setChecked(False)
                return
        else:
            try:
                self.run_attack_pub.publish(Int32(0))
            except:
                self.ui.attackButton.setChecked(True)
                return

        # For when the button is set via direct method call, not by event call
        self.ui.attackButton.setChecked(checked)

    # Called by attackMode combobox when there is a change in the selected index.
    def attackMode(self, index):
        if self.ui.attackButton.isChecked():
            try:
                self.run_attack_pub.publish(Int32(index+1)) # Start base index at 1
            except:
                return

    # Called by sensor checkboxes when there is a check or uncheck event.
    def attackSensor(self):
        if self.ui.attackButton.isChecked():
            try:
                self.sensor_attack_pub.publish(Int32(self.getAttackSensorValue()))
            except:
                return

    # Determine the appropriate binary-style representation for which sensor checkboxes are checked.
    def getAttackSensorValue(self):
        value = 0
        if self.ui.gpsCheckBox.isChecked():
            value += 4
        if self.ui.enclCheckBox.isChecked():
            value += 2
        if self.ui.encrCheckBox.isChecked():
            value += 1
        return value

    def updateInputPlot(self):
        """ Redraws the input plot
        """
        self.inputPlotBase.setData(self.in_Base)
        self.inputPlotRef.setData(self.in_Ref)

    def updateOutputPlot(self):
        """ Redraws the output plot
        """
        self.outputPlotGPS.setData(self.out_GPS)
        self.outputPlotLE.setData(self.out_EncL)
        self.outputPlotRE.setData(self.out_EncR)

    def updateRightPlot(self):
        """ Redraws the righthand plot
        """
        self.rightPlotOdom.setData(self.out_Odom)
        if self.ui.rcButton.isChecked():
            self.resilientControllerRegions[-1].setRegion((self.resilientControllerRegions[-1].getRegion()[0], len(self.out_Odom)))
        for region in self.resilientControllerRegions:
            regionVals = region.getRegion()
            if regionVals[1] == 0:
                self.resilientControllerRegions.remove(region)
                self.ui.rightPlot.removeItem(region)
            if len(self.out_Odom) == 300:
                region.setRegion((regionVals[0] - 1, regionVals[1] - 1))
        self.rightPlotRef.setData(self.in_Ref)

    def setLandsharkSpeed(self):
        self.desired_speed_pub.publish(Float32(float(self.ui.desiredSpeedEdit.text())))

    def setKP(self):
        self.kp_pub.publish(Float32(float(self.ui.kpEdit.text())))

    def setKI(self):
        self.ki_pub.publish(Float32(float(self.ui.kiEdit.text())))

    def setTrimLeft(self):
        # Get current trim value
        trim = float(self.ui.trimValueLabel.text())

        # Decrement trim value
        trim -= self.trimIncrement

        # Display updated trim value
        self.ui.trimValueLabel.setText(str(trim))

        # Publish new trim value
        self.trim_pub.publish(Float32(trim))

    def setTrimRight(self):
        # Get current trim value
        trim = float(self.ui.trimValueLabel.text())

        # Increment trim value
        trim += self.trimIncrement

        # Display updated trim value
        self.ui.trimValueLabel.setText(str(trim))

        # Publish new trim value
        self.trim_pub.publish(Float32(trim))

    def addWaypoint(self):
        latitude, longitude, ok = ui.geocoord.GeoCoordDialog.getLatLong()
        if ok:
            self.ui.navView.scene().addWaypoint(Waypoint(latitude, longitude))

    def removeWaypoint(self):
        #self.waypointList.remove(self.ui.waypointListView.selectedItem)
        #self.waypoints.remove(way)
        return
        
    def clearWaypoints(self):
        self.ui.navView.scene().clearWaypoints()

    def uploadWaypoints(self):
        # wait for action server
        #self.actionClient.wait_for_server()
        
        # build waypoints goal
        goal = NavigateToWayPointsActionGoal()
        goal.goal.way_point_type = NavigateToWayPointsGoal.GPS
        #goal.way_point_type = NavigateToWayPointsGoal.RELATIVE_TO_ROBOT_XY_NE
        for way in self.ui.navView.scene().waypoints:
            goal.goal.way_point_list.append(Point(float(way.coordinate.longitude()), float(way.coordinate.latitude()), float(0.0)))

        # Send waypoints
        self.waypoints_pub.publish(goal)
        #self.actionClient.send_goal(goal)
        
        #self.actionResultThread = threading.Thread(target = self.waitForActionResult)
        #self.actionResultThread.setDaemon(True)
        #self.actionResultThread.start()
        
    def waitForActionResult(self):
        self.actionClient.wait_for_result()
        result = self.actionClient.get_result()
        
        if result == None:
            print 'result: None'
            return
            
        if result.status == NavigateToWayPointsResult.SUCCESS:
            print 'result: SUCCESS'
        
        #self.lastVisitedWaypoint = -1

    def start_landshark_comm(self):
        # Initialize ROS node
        rospy.init_node('landshark_demo', disable_signals=True)

        # Subscribe to HACMS Demo topics
        self.base_sub = rospy.Subscriber("/landshark_demo/base_vel", TwistStamped, self.captureBase)
        self.ref_sub = rospy.Subscriber("/landshark_demo/ref_vel", TwistStamped, self.captureRef)
        self.est_sub = rospy.Subscriber("/landshark_demo/est_vel", TwistStamped, self.captureEst)
        self.odom_sub = rospy.Subscriber("/landshark_demo/odom", Odometry, self.captureOdom)
        self.encL_sub = rospy.Subscriber("/landshark_demo/left_enc_vel", TwistStamped, self.captureEncL)
        self.encR_sub = rospy.Subscriber("/landshark_demo/right_enc_vel", TwistStamped, self.captureEncR)
        self.gps_sub = rospy.Subscriber("/landshark_demo/gps_vel", TwistStamped, self.captureGPS)
        self.action_sub = rospy.Subscriber("/landshark_waypoint_navigation/feedback", NavigateToWayPointsActionFeedback, self.captureActionFeedback)
        #self.actionClient = actionlib.SimpleActionClient('/landshark_waypoint_navigation', NavigateToWayPointsAction)
        #self.actionResultThread = threading.Thread()
        #self.actionResultThread.setDaemon(True)
        
        # Publish to HACMS Demo topics
        self.desired_speed_pub = rospy.Publisher('/landshark_demo/desired_speed', Float32)
        self.trim_pub = rospy.Publisher('/landshark_demo/trim', Float32)
        self.kp_pub = rospy.Publisher('/landshark_demo/kp', Float32)
        self.ki_pub = rospy.Publisher('/landshark_demo/ki', Float32)
        self.run_rc_pub = rospy.Publisher('/landshark_demo/run_rc', Int32)
        self.run_attack_pub = rospy.Publisher('/landshark_demo/run_attack', Int32)
        self.sensor_attack_pub = rospy.Publisher('/landshark_demo/sensor_attack', Int32)
        self.waypoints_pub = rospy.Publisher('/landshark_waypoint_navigation/ocu_goal', NavigateToWayPointsActionGoal)

        self.pubsubs = [
            self.base_sub,
            self.ref_sub,
            self.est_sub,
            self.odom_sub,
            self.encL_sub,
            self.encR_sub,
            self.gps_sub,
            self.action_sub,
            self.desired_speed_pub,
            self.trim_pub,
            self.kp_pub,
            self.ki_pub,
            self.run_rc_pub,
            self.run_attack_pub,
            self.sensor_attack_pub,
            self.waypoints_pub
        ]

        return True

    def stop_landshark_comm(self):
        # Unregister HACMS Demo topics
        for topic in self.pubsubs:
            topic.unregister()
        return True

    def captureBase(self, msg):
        self.in_Base.append(msg.twist.linear.x)

    def captureRef(self, msg):
        self.in_Ref.append(msg.twist.linear.x)

    def captureEst(self, msg):
        self.ui.estimatedSpeedLCD.display(msg.twist.linear.x)

    def captureOdom(self, msg):
        self.ui.actualSpeedLCD.display(msg.twist.twist.linear.x)
        self.out_Odom.append(msg.twist.twist.linear.x)

    def captureEncL(self, msg):
        self.out_EncL.append(msg.twist.linear.x)

    def captureEncR(self, msg):
        self.out_EncR.append(msg.twist.linear.x)

    def captureGPS(self, msg):
        self.out_GPS.append(msg.twist.linear.x)

    def captureActionFeedback(self, msg):
        #if feedback.feedback.current_way_point_idx == self.lastVisitedWaypoint + 2:
        feedbackText = 'Action Feedback:\n'

def main():
    app = QApplication(sys.argv)
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        import debug
        h = debug.HACMSDebugWindow()
    else:
        import demo
        h = demo.HACMSDemoWindow()
    h.show()
    app.aboutToQuit.connect(h.fileQuit)
    app.lastWindowClosed.connect(app.quit)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

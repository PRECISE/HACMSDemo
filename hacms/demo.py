#!/usr/bin/env python

import sys, string
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
#from PySide.QtGui import *
from PyQt4 import QtGui, Qwt5
import matplotlib
#matplotlib.use('Qt4Agg')
#matplotlib.rcParams['backend.qt4']='PySide'
import pylab
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# HACMS Python modules
from remote import Remote
import ui

class HACMSDemoWindow(QtGui.QMainWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.actionAbout.triggered.connect(self.about)
        self.ui.actionQuit.triggered.connect(self.fileQuit)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.setSpeedButton.clicked.connect(self.setLandsharkSpeed)
        self.dpi = 100
        self.outFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.outCanvas = FigureCanvas(self.outFig)
        self.outCanvas.setParent(self.ui.outputPlot)
        self.outAxes = self.outFig.add_subplot(111)
        self.inFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.inCanvas = FigureCanvas(self.inFig)
        self.inCanvas.setParent(self.ui.inputPlot)
        self.inAxes = self.inFig.add_subplot(111)
#         self.outPlot = Qwt5.QwtPlot()
#         self.outPlot.setParent(self.ui.outputPlot)
#         self.outPlotCurve = Qwt5.QwtPlotCurve()
#         self.outPlotCurve.attach(self.outPlot)
        self.in_Odom = []
        self.out_EncL = []
        self.out_EndR = []
        self.out_GPS = []
        self.out_Odom = []
        #TODO: Try to look into flushing queue as it gets backlogged (CPU overloaded...)
        #TODO: Add save figure capabilities
        #TODO: Layout widgets so that the console and plots will resize with the window
        self.remote = Remote(self.ui.console)

    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")
                
    def fileQuit(self):
        if self.ui.landsharkButton.isChecked():
            self.stop_landshark_comm()
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()
                
    def enableAllElements(self):
        self.ui.ccButton.setEnabled(True)
        self.ui.rcButton.setEnabled(True)
        self.ui.attackButton.setEnabled(True)
        self.ui.actualLabel.setEnabled(True)
        self.ui.estimatedLabel.setEnabled(True)
        self.ui.desiredSpeedLabel.setEnabled(True)
        self.ui.desiredSpeedEdit.setEnabled(True)
        self.ui.setSpeedButton.setEnabled(True)
        self.ui.actualSpeedLabel.setEnabled(True)
        self.ui.actualSpeedLCD.setEnabled(True)
        self.ui.estimatedSpeedLabel.setEnabled(True)
        self.ui.estimatedSpeedLCD.setEnabled(True)
        self.ui.outputPlotLabel.setEnabled(True)
        self.ui.outputPlotLabel.setEnabled(True)
        self.ui.inputPlotLabel.setEnabled(True)
        self.ui.inputPlotLabel.setEnabled(True)
        self.in_Odom = []
        self.out_EncL = []
        self.out_EndR = []
        self.out_GPS = []
        self.out_Odom = []
        
    def disableAllElements(self):
        self.cc(False)
        self.rc(False)
        self.attack(False)    
        self.ui.ccButton.setEnabled(False)
        self.ui.rcButton.setEnabled(False)
        self.ui.attackButton.setEnabled(False)
        self.ui.actualLabel.setEnabled(False)
        self.ui.estimatedLabel.setEnabled(False)
        self.ui.desiredSpeedLabel.setEnabled(False)
        self.ui.desiredSpeedEdit.setEnabled(False)
        self.ui.setSpeedButton.setEnabled(False)
        self.ui.actualSpeedLabel.setEnabled(False)
        self.ui.actualSpeedLCD.setEnabled(False)
        self.ui.estimatedSpeedLabel.setEnabled(False)
        self.ui.estimatedSpeedLCD.setEnabled(False)
        self.ui.outputPlotLabel.setEnabled(False)
        self.ui.outputPlotLabel.setEnabled(False)
        self.ui.inputPlotLabel.setEnabled(False)
        self.ui.inputPlotLabel.setEnabled(False)

    def landshark(self, checked):
        if checked:
            res = self.remote.startLandshark()
            self.start_landshark_comm()
            self.enableAllElements()
        else:
            self.disableAllElements()
            self.stop_landshark_comm()
            res = self.remote.stopLandshark()
        self.ui.landsharkButton.setChecked(res)

    def cc(self, checked):
        if checked:
            res = self.remote.startCC()
        else:
            res = self.remote.stopCC()           
        self.ui.ccButton.setChecked(res)

    def rc(self, checked):
        if checked:
            try:
                self.run_rc_pub.publish(Bool(True))
            except:
                self.ui.rcButton.setChecked(False)
        else:
            try:
                self.run_rc_pub.publish(Bool(False))
            except:
                self.ui.rcButton.setChecked(True)
        self.ui.rcButton.setChecked(checked)

    def attack(self, checked):
        if checked:
            try:
            	mode = 0
#             	if self.ui.attack1RadioButton.isChecked:
#             		mode = 1
#             	elif self.ui.attack2RadioButton.isChecked:
#             		mode = 2
#             	elif self.ui.attack3RadioButton.isChecked:
#             		mode = 3
				#TODO: Add radio button group to GUI
            	self.run_attack_pub.publish(Int32(mode))
            except:
                self.ui.attackButton.setChecked(False)
        else:
            try:
                self.run_attack_pub.publish(Int32(0))
            except:
                self.ui.attackButton.setChecked(True)
        self.ui.attackButton.setChecked(checked)

    def getWidgetColor(self, widget):
        style = widget.styleSheet()
        if "background-color: green;" in style:
            return "green"
        if "background-color: red;" in style:
            return "red"

    def toggleWidgetColor(self, widget):
        style = widget.styleSheet()
        if self.getWidgetColor(widget) is "green":
            widget.setStyleSheet(string.replace(style, "background-color: green;", "background-color: red;"))
        elif self.getWidgetColor(widget) is "red":
            widget.setStyleSheet(string.replace(style, "background-color: red;", "background-color: green;"))

    def updateActualSpeedLCD(self, msg):
        self.ui.actualSpeedLCD.display(msg.twist.twist.linear.x)

    def updateEstimatedSpeedLCD(self, msg):
        self.ui.estimatedSpeedLCD.display(msg.twist.linear.x)

#     def save_plot(self):
#         file_choices = "PNG (*.png)|*.png"
#         
#         path = unicode(QFileDialog.getSaveFileName(self, 
#                         'Save file', '', 
#                         file_choices))
#         if path:
#             self.outCanvas.print_figure(path, dpi=self.dpi)
#             #self.statusBar().showMessage('Saved to %s' % path, 2000)

    def draw_outputPlot(self):
        """ Redraws the output plot
        """
        # clear the axes and redraw the plot anew
        #
        self.outAxes.clear()        
        self.outAxes.grid(True)
        self.outAxes.plot(self.out_Odom)
        self.outCanvas.draw()

    def setLandsharkSpeed(self):
        msg = TwistStamped()
        msg.twist.linear.x = float(self.ui.desiredSpeedEdit.text())
        self.desired_speed_pub.publish(msg)

    def start_landshark_comm(self):
        # Initialize ROS node
        rospy.init_node('landshark_demo', disable_signals=True)

        # Subscribe to HACMS Demo topics
        self.odom_sub = rospy.Subscriber("/landshark_demo/odom", Odometry, self.gatherOdom)
        self.gps_sub = rospy.Subscriber("/landshark_demo/gps_velocity", TwistStamped, self.gatherGPS)

        #self.desired_speed_pub = rospy.Publisher('/landshark_demo/desired_speed', TwistStamped)
        self.desired_speed_pub = rospy.Publisher('/landshark_control/base_velocity', TwistStamped)
        self.run_cc_pub = rospy.Publisher('/landshark_demo/run_cc', Bool)
        self.run_rc_pub = rospy.Publisher('/landshark_demo/run_rc', Bool)
        self.run_attack_pub = rospy.Publisher('/landshark_demo/run_attack', Int32)

        return True
        
    def stop_landshark_comm(self):
        # Unsubscribe to HACMS Demo topics
        self.odom_sub.unregister()
        self.gps_sub.unregister()

        self.desired_speed_pub.unregister()
        self.run_cc_pub.unregister()
        self.run_rc_pub.unregister()
        self.run_attack_pub.unregister()

        #rospy.signal_shutdown("Turning off ROSPy")

        return True
        
    def gatherOdom(self, msg):
        self.updateActualSpeedLCD(msg)
        self.out_Odom.append(msg.twist.twist.linear.x)
#         self.outPlotCurve.setData(self.out_Odom, range(len(self.out_Odom)))
#         self.outPlot.replot()
        self.draw_outputPlot()
        
    def gatherGPS(self, msg):
        self.updateEstimatedSpeedLCD(msg)


def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    app.aboutToQuit.connect(h.fileQuit)
    app.lastWindowClosed.connect(app.quit)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


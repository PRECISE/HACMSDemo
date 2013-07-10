#!/usr/bin/env python

import sys, string
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, TwistStamped
#from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from PySide.QtGui import *
import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'
import pylab
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# HACMS Python modules
from remote import Remote
import ui

class HACMSDemoWindow(QMainWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.actionAbout.triggered.connect(self.about)
        self.ui.actionQuit.triggered.connect(self.close)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.setSpeedButton.clicked.connect(self.save_plot)
        self.dpi = 100
        self.outFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.outCanvas = FigureCanvas(self.outFig)
        self.outCanvas.setParent(self.ui.outputPlot)
        self.outAxes = self.outFig.add_subplot(111)
        self.inFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.inCanvas = FigureCanvas(self.inFig)
        self.inCanvas.setParent(self.ui.inputPlot)
        self.inAxes = self.inFig.add_subplot(111)
        #TODO: Add save figure capabilities
        self.inDataOdom = []
        self.outDataOdom = []
        self.remote = Remote(self.ui.console)

    def about(self):
        QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")
                
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
            self.landshark_comm()
            self.enableAllElements()
        else:
            res = self.remote.stopLandshark()
            #TODO: add proper shutdown handling   rospy.signal_shutdown("Turning off Landshark")
            self.disableAllElements()

        self.ui.landsharkButton.setChecked(res)

    def cc(self, checked):
        if checked:
            try:
                self.run_cc_pub.publish(Bool(True))
            except:
                self.ui.ccButton.setChecked(False)
        else:
            try:
                self.run_cc_pub.publish(Bool(False))
            except:
                self.ui.ccButton.setChecked(True)

    def rc(self, checked):
        self.ui.rcButton.setChecked(self.landshark_comm() if checked else False)
        #self.ui.rcButton.setChecked(self.remote.startRC() if checked else self.remote.stopRC())

    def attack(self, checked):
        if checked:
            try:
                self.run_attack_pub.publish(Bool(True))
            except:
                self.ui.attackButton.setChecked(False)
        else:
            try:
                self.run_attack_pub.publish(Bool(False))
            except:
                self.ui.attackButton.setChecked(True)

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

    def updateOutputPlot(self, msg, encL=False, encR=False, gps=False, odom=False):
        if encL:
            return
        if encR:
            return
        if gps:
            return
        if odom:
            self.ui.estimatedSpeedLCD.display(msg.twist.twist.linear.x)
            #self.on_draw()
            
    def gatherOdom(self, msg):
        self.updateActualSpeedLCD(msg)
        self.inDataOdom.append(msg.twist.twist.linear.x)
        self.on_draw()
            
    def save_plot(self):
        file_choices = "PNG (*.png)|*.png"
        
        path = unicode(QFileDialog.getSaveFileName(self, 
                        'Save file', '', 
                        file_choices))
        if path:
            self.outCanvas.print_figure(path, dpi=self.dpi)
            #self.statusBar().showMessage('Saved to %s' % path, 2000)
            
    def on_draw(self):
        """ Redraws the figure
        """
        # clear the axes and redraw the plot anew
        #
        self.outAxes.clear()        
        self.outAxes.grid(True)
        
        self.outAxes.plot(self.inDataOdom)
        
        self.outCanvas.draw()

    def setLandsharkSpeed(self):
        msg = TwistStamped()
        msg.twist.linear.x = float(self.ui.desiredSpeedEdit.text())
        self.desired_speed_pub.publish(msg)
        self.test_pub.publish("test")

    def landshark_comm(self):
        # Initialize ROS node
        rospy.init_node('landshark_demo', disable_signals=True)

        # Subscribe to HACMS Demo topics
        rospy.Subscriber("/landshark_demo/odom", Odometry, self.gatherOdom)
        #rospy.Subscriber("/landshark_demo/gps_velocity", TwistStamped, self.updateEstimatedSpeedLCD)

#         self.desired_speed_pub = rospy.Publisher('/landshark_demo/desired_speed', TwistStamped)
#         self.run_cc_pub = rospy.Publisher('/landshark_demo/run_cc', Bool)
#         self.run_rc_pub = rospy.Publisher('/landshark_demo/run_rc', Bool)
#         self.run_attack_pub = rospy.Publisher('/landshark_demo/run_attack', Bool)
#         self.test_pub = rospy.Publisher('/landshark_demo/test', String)

        #TODO: stop subscribers just as the GUI is closed (to prevent bad callback)

        return True


def main():
    app = QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


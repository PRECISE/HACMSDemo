#!/usr/bin/env python

# Standard Python modules
import sys, string
from collections import deque

#ROSPy modules
import rospy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# QT modules
from PyQt4.QtGui import *
from PyQt4.QtCore import *

# Matplotlib modules
import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# HACMS modules
from remote import Remote
import ui.images_rc
from navigation import MapnikScene
from mapview import MapView

        #TODO: Try to look into flushing queue as it gets backlogged (CPU overloaded...)
        #TODO: Layout widgets so that the console and plots will resize with the window
        #TODO: Add legend for plot lines, fit titles and axes labels, too
        #TODO: Use timestamps for x-axis, data will be plotted accordingly
        #TODO: Add navigation tab with Google Maps
        #TODO: Combine three plots into a single plot (using subplots)?
        #TODO: Change big buttons to be darker or colored when checked
        #TODO: Put all main widgets into a list
        #TODO: put red line to mark attack moment or attack region
        #TODO: Make separate debug and demo UIs (make parent window class, subclass it for each type)
        #TODO: Ability to save plots showing the entire data set, save data in Matlab-friendly format (write to a file in real-time? file name and location?)
        #TODO: Ability to save/transfer .bag files (for video)
        #TODO: Fix plots so that the titles and axes labels are shown completely
        #TODO: Navigation tab
        #TODO: Change attack mode whenever attack radio button changes (if Attack button is pressed)
        #TODO: Update GUI tabstop order
     
class HACMSWindow(QMainWindow):
    def __init__(self):
        super(HACMSWindow, self).__init__()
        
    def init_window(self):
        self.ui.setupUi(self)
        self.init_widgets()
        self.init_data_structs()
        
    def init_widgets(self):
        self.remote = Remote(self.ui.console)
        self.widgets = [
            self.ui.ccButton, 
            self.ui.rcButton, 
            self.ui.attackButton, 
            self.ui.attack1RadioButton, 
            self.ui.attack2RadioButton, 
            self.ui.attack3RadioButton, 
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
            self.ui.saveInputPlotButton,
            self.ui.saveOutputPlotButton,
            self.ui.saveRightPlotButton,
        ]
        #self.ui.mapView.setScene(MapnikScene(self.ui.mapView))
        #self.ui.mapView = MapView(self.ui.mapWidget)
        self.init_signals()
        self.init_plots()
        #self.init_waypoints()

    def init_signals(self):
        self.ui.actionAbout.triggered.connect(self.about)
        self.ui.actionQuit.triggered.connect(self.fileQuit)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.attack1RadioButton.toggled.connect(self.attack1)
        self.ui.attack2RadioButton.toggled.connect(self.attack2)
        self.ui.attack3RadioButton.toggled.connect(self.attack3)
        self.ui.saveButton.toggled.connect(self.saveData)
        self.ui.setSpeedButton.clicked.connect(self.setLandsharkSpeed)
        self.ui.setKPButton.clicked.connect(self.setKP)
        self.ui.setKIButton.clicked.connect(self.setKI)
        self.ui.setTrimLeftButton.clicked.connect(self.setTrimLeft)
        self.ui.setTrimRightButton.clicked.connect(self.setTrimRight)
        self.ui.saveInputPlotButton.clicked.connect(self.save_inputPlot)
        self.ui.saveOutputPlotButton.clicked.connect(self.save_outputPlot)
        self.ui.saveRightPlotButton.clicked.connect(self.save_rightPlot)
        
        # Set Validator for parameter fields
        self.validator = QDoubleValidator()
        self.validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.desiredSpeedEdit.setValidator(self.validator)
        self.ui.kpEdit.setValidator(self.validator)
        self.ui.kiEdit.setValidator(self.validator)
        
    def init_data_structs(self):
        self.trimIncrement = 0.001
        self.windowSize = 300
        self.in_Base = deque(maxlen=self.windowSize)
        self.in_Ref = deque(maxlen=self.windowSize)
        self.out_Odom = deque(maxlen=self.windowSize)
        self.out_EncL = deque(maxlen=self.windowSize)
        self.out_EncR = deque(maxlen=self.windowSize)
        self.out_GPS = deque(maxlen=self.windowSize)
        self.out_Trim = deque(maxlen=self.windowSize)
        
    def init_plots(self):
        self.dpi = 100
        self.y_top = 1.5
        self.inFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.inCanvas = FigureCanvas(self.inFig)
        self.inCanvas.setParent(self.ui.inputPlot)
        self.inAxes = self.inFig.add_subplot(111)
        self.inAxes.grid(True)
        self.inAxes.set_ybound(0, self.y_top)
        self.inAxes.set_autoscaley_on(False)
#         self.inAxes.set_xlabel('time')
#         self.inAxes.set_ylabel('speed')
#         self.inAxes.set_title('Input')
        self.outFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.outCanvas = FigureCanvas(self.outFig)
        self.outCanvas.setParent(self.ui.outputPlot)
        self.outAxes = self.outFig.add_subplot(111)
        self.outAxes.grid(True)
        self.outAxes.set_ybound(0, self.y_top)
#         self.outAxes.set_autoscaley_on(False)
#         self.outAxes.set_xlabel('time')
#         self.outAxes.set_ylabel('speed')
#         self.outAxes.set_title('Output')
        self.rightFig = Figure((4.21, 4.41), dpi=self.dpi)
        self.rightCanvas = FigureCanvas(self.rightFig)
        self.rightCanvas.setParent(self.ui.rightPlot)
        self.rightAxes = self.rightFig.add_subplot(111)
        self.rightAxes.grid(True)
        self.rightAxes.set_ybound(0, self.y_top)
        self.rightAxes.set_autoscaley_on(False)
#         self.rightAxes.set_xlabel('time')
#         self.rightAxes.set_ylabel('speed')
#         self.rightAxes.set_title('Odometry')

    def init_waypoints(self):
        self.waypointList = QStringList()
        self.waypointList.append("test")
        self.waypointModel = QStringListModel()
        self.ui.waypointListView.setModel(self.waypointModel)
        self.waypointModel.setStringList(self.waypointList)

    def about(self):
        QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application allows for control of the LandShark "
                "robot while displaying live ROS telemetry data.\n\nDeveloped by the "
                "PRECISE Lab at Penn.")
                
    def fileQuit(self):
        if self.ui.landsharkButton.isChecked():
            self.stop_landshark_comm()
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()
        
    def zeroData(self):
        self.in_Base.clear()
        self.in_Ref.clear()
        self.out_Odom.clear()
        self.out_EncL.clear()
        self.out_EncR.clear()
        self.out_GPS.clear()
        self.out_Trim.clear()
                
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
        else:
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
                mode = 0
                if self.ui.attack1RadioButton.isChecked():
                    mode = 1
                elif self.ui.attack2RadioButton.isChecked():
                    mode = 2
                elif self.ui.attack3RadioButton.isChecked():
                    mode = 3
                self.run_attack_pub.publish(Int32(mode))
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
        
    def attack1(self, checked):
        if checked and self.ui.attackButton.isChecked():
            try:
                self.run_attack_pub.publish(Int32(1))
            except:
                return
    
    def attack2(self, checked):
        if checked and self.ui.attackButton.isChecked():
            try:
                self.run_attack_pub.publish(Int32(2))
            except:
                return
                
    def attack3(self, checked):
        if checked and self.ui.attackButton.isChecked():
            try:
                self.run_attack_pub.publish(Int32(3))
            except:
                return

    def getWidgetColor(self, widget):
        style = widget.styleSheet()
        if "background-color: green;" in style:
            return "green"
        if "background-color: red;" in style:
            return "red"

    def toggleWidgetColor(self, widget, setColor=None):
        style = widget.styleSheet()
        if setColor is None:
            if self.getWidgetColor(widget) is "green":
                widget.setStyleSheet(string.replace(style, "background-color: green;", "background-color: red;"))
            elif self.getWidgetColor(widget) is "red":
                widget.setStyleSheet(string.replace(style, "background-color: red;", "background-color: green;"))
        else:
            if setColor is "red":
                widget.setStyleSheet(string.replace(style, "background-color: green;", "background-color: red;"))
            elif setColor is "green":
                widget.setStyleSheet(string.replace(style, "background-color: red;", "background-color: green;"))

    def updateActualSpeedLCD(self, value):
        self.ui.actualSpeedLCD.display(value)

    def updateEstimatedSpeedLCD(self, value):
        self.ui.estimatedSpeedLCD.display(value)

    def save_plot(self):
        file_choices = "PNG (*.png)|*.png"
        return unicode(QFileDialog.getSaveFileName(self, 'Save file', '', file_choices))

    def draw_inputPlot(self):
        """ Redraws the input plot
        """
        # clear the axes and redraw the plot anew
        #
        self.inAxes.clear()
        self.inAxes.grid(True)
        self.inAxes.set_ybound(0, self.y_top)
        self.inAxes.set_autoscaley_on(False) 
#         self.inAxes.set_xlabel('time')
#         self.inAxes.set_ylabel('speed')
#         self.inAxes.set_title('Input')
        self.inAxes.plot(self.in_Base, 'r', linewidth=2)
        self.inAxes.plot(self.in_Ref, 'c', linewidth=2)
        self.inCanvas.draw()
        
    def save_inputPlot(self):
        path = self.save_plot()
        if path:
            self.inCanvas.print_figure(path, dpi=self.dpi)
            self.statusBar().showMessage('Saved to %s' % path, 2000)
    
    def draw_outputPlot(self):
        """ Redraws the output plot
        """
        # clear the axes and redraw the plot anew
        #
        self.outAxes.clear()
        self.outAxes.grid(True)
        self.outAxes.set_ybound(0, self.y_top)
#         self.outAxes.set_autoscaley_on(False) 
#         self.outAxes.set_xlabel('time')
#         self.outAxes.set_ylabel('speed')
#         self.outAxes.set_title('Output')
        self.outAxes.plot(self.out_EncL, 'b', linewidth=2)
        self.outAxes.plot(self.out_EncR, 'g', linewidth=2)
        self.outAxes.plot(self.out_GPS, 'm', linewidth=2)
        self.outCanvas.draw()
        
    def save_outputPlot(self):
        path = self.save_plot()
        if path:
            self.outCanvas.print_figure(path, dpi=self.dpi)
            self.statusBar().showMessage('Saved to %s' % path, 2000)
        
    def draw_rightPlot(self):
        """ Redraws the righthand plot
        """
        # clear the axes and redraw the plot anew
        #
        self.rightAxes.clear()
        self.rightAxes.grid(True)
        self.rightAxes.set_ybound(0, self.y_top)
        self.rightAxes.set_autoscaley_on(False) 
#         self.rightAxes.set_xlabel('time')
#         self.rightAxes.set_ylabel('speed')
#         self.rightAxes.set_title('Odometry')
        self.rightAxes.plot(self.out_Odom, 'b', linewidth=2)
        self.rightAxes.plot(self.in_Ref, 'c', linewidth=2)
        self.rightCanvas.draw()
    
    def save_rightPlot(self):
        path = self.save_plot()
        if path:
            self.rightCanvas.print_figure(path, dpi=self.dpi)
            self.statusBar().showMessage('Saved to %s' % path, 2000)

    def setLandsharkSpeed(self):
        self.desired_speed_pub.publish(Float32(float(self.ui.desiredSpeedEdit.text())))
    
    def setKP(self):
        self.kp_pub.publish(Float32(float(self.ui.kpEdit.text())))
        
    def setKI(self):
        self.ki_pub.publish(Float32(float(self.ui.kiEdit.text())))
        
    def setAutotrim(self):
        self.autotrim_pub.publish(Float32(float(self.ui.autotrimEdit.text())))
    
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

        # Publish to HACMS Demo topics
        self.desired_speed_pub = rospy.Publisher('/landshark_demo/desired_speed', Float32)
        self.autotrim_pub = rospy.Publisher('/landshark_demo/autotrim', Float32)
        self.trim_pub = rospy.Publisher('/landshark_demo/trim', Float32)
        self.kp_pub = rospy.Publisher('/landshark_demo/kp', Float32)
        self.ki_pub = rospy.Publisher('/landshark_demo/ki', Float32)
        self.run_rc_pub = rospy.Publisher('/landshark_demo/run_rc', Int32)
        self.run_attack_pub = rospy.Publisher('/landshark_demo/run_attack', Int32)

        return True
        
    def stop_landshark_comm(self):
        # Unregister HACMS Demo subscribed topics
        self.base_sub.unregister()
        self.ref_sub.unregister()
        self.est_sub.unregister()
        self.odom_sub.unregister()
        self.encL_sub.unregister()
        self.encR_sub.unregister()
        self.gps_sub.unregister()

        # Unregister HACMS Demo published topics
        self.desired_speed_pub.unregister()
        self.autotrim_pub.unregister()
        self.trim_pub.unregister()
        self.kp_pub.unregister()
        self.ki_pub.unregister()
        self.run_rc_pub.unregister()
        self.run_attack_pub.unregister()

        rospy.signal_shutdown("Turning off ROSPy")

        return True

    def captureBase(self, msg):
        self.in_Base.append(msg.twist.linear.x)
        self.draw_inputPlot()

    def captureRef(self, msg):
        self.in_Ref.append(msg.twist.linear.x)

    def captureEst(self, msg):
        self.updateEstimatedSpeedLCD(msg.twist.linear.x)

    def captureOdom(self, msg):
        self.updateActualSpeedLCD(msg.twist.twist.linear.x)
        self.out_Odom.append(msg.twist.twist.linear.x)
        self.draw_rightPlot()
#         self.out_Trim.append(msg.pose.pose.position.y)
#         self.draw_trimPlot()
        
    def captureEncL(self, msg):
        self.out_EncL.append(msg.twist.linear.x)
        self.out_GPS.append(msg.twist.linear.y)
        self.draw_outputPlot()
        
    def captureEncR(self, msg):
        self.out_EncR.append(msg.twist.linear.x)
        
    def captureGPS(self, msg):
        #self.out_GPS.append(msg.twist.linear.x)
        return

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

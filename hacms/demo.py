#!/usr/bin/env python

import sys, string
from collections import deque
import rospy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
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

        #TODO: Try to look into flushing queue as it gets backlogged (CPU overloaded...)
        #TODO: Layout widgets so that the console and plots will resize with the window
        #TODO: Add legend for plot lines, fit titles and axes labels, too
        #TODO: Use timestamps for x-axis, data will be plotted accordingly
        #TODO: Add navigation tab with Google Maps
        #TODO: Combine three plots into a single plot (using subplots)?
        #TODO: Change big buttons to be darker or colored when checked
        #TODO: Put all main widgets into a list
        #TODO: put red line to mark attack moment or attack region
        
class HACMSDemoWindow(QtGui.QMainWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_signals()
        self.init_data_structs()
        self.init_plots()
        self.remote = Remote(self.ui.console)
        #self.mainWidgets = [self.ui.ccButton]
        
    def init_signals(self):
        self.ui.actionAbout.triggered.connect(self.about)
        self.ui.actionQuit.triggered.connect(self.fileQuit)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.setSpeedButton.clicked.connect(self.setLandsharkSpeed)
        self.ui.setKPButton.clicked.connect(self.setKP)
        self.ui.setKIButton.clicked.connect(self.setKI)
        self.ui.setAutotrimButton.clicked.connect(self.setAutotrim)
        self.ui.setTrimLeftButton.clicked.connect(self.setTrimLeft)
        self.ui.setTrimRightButton.clicked.connect(self.setTrimRight)
        self.ui.saveInputPlotButton.clicked.connect(self.save_inputPlot)
        self.ui.saveOutputPlotButton.clicked.connect(self.save_outputPlot)
        self.ui.saveRightPlotButton.clicked.connect(self.save_rightPlot)
        self.validator = QtGui.QDoubleValidator()
        self.validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
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
        self.inAxes.set_xlabel('time')
        self.inAxes.set_ylabel('speed')
        self.inAxes.set_title('Input')
        self.outFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.outCanvas = FigureCanvas(self.outFig)
        self.outCanvas.setParent(self.ui.outputPlot)
        self.outAxes = self.outFig.add_subplot(111)
        self.outAxes.grid(True)
        self.outAxes.set_ybound(0, self.y_top)
        self.outAxes.set_autoscaley_on(False)
        self.outAxes.set_xlabel('time')
        self.outAxes.set_ylabel('speed')
        self.outAxes.set_title('Output')
        self.rightFig = Figure((4.21, 4.41), dpi=self.dpi)
        self.rightCanvas = FigureCanvas(self.rightFig)
        self.rightCanvas.setParent(self.ui.rightPlot)
        self.rightAxes = self.rightFig.add_subplot(111)
        self.rightAxes.grid(True)
        self.rightAxes.set_ybound(0, self.y_top)
        self.rightAxes.set_autoscaley_on(False)
        self.rightAxes.set_xlabel('time')
        self.rightAxes.set_ylabel('speed')
        self.rightAxes.set_title('Odometry')

    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application allows for control of the LandShark "
                "robot while displaying live ROS telemetry data.")
                
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
                
    def enableAllElements(self):
#         for widget in self.mainWidgets:
#             widget.setEnabled(True)
        
        self.ui.ccButton.setEnabled(True)
        self.ui.rcButton.setEnabled(True)
        self.ui.attackButton.setEnabled(True)
        self.ui.actualLabel.setEnabled(True)
        self.ui.estimatedLabel.setEnabled(True)
        self.ui.desiredSpeedLabel.setEnabled(True)
        self.ui.desiredSpeedEdit.setEnabled(True)
        self.ui.setSpeedButton.setEnabled(True)
        self.ui.kpLabel.setEnabled(True)
        self.ui.kpEdit.setEnabled(True)
        self.ui.setKPButton.setEnabled(True)
        self.ui.kiLabel.setEnabled(True)
        self.ui.kiEdit.setEnabled(True)
        self.ui.setKIButton.setEnabled(True)
        self.ui.autotrimLabel.setEnabled(True)
        self.ui.autotrimEdit.setEnabled(True)
        self.ui.setAutotrimButton.setEnabled(True)
        self.ui.trimLabel.setEnabled(True)
        self.ui.trimValueLabel.setEnabled(True)
        self.ui.setTrimLeftButton.setEnabled(True)
        self.ui.setTrimRightButton.setEnabled(True)
        self.ui.actualSpeedLabel.setEnabled(True)
        self.ui.actualSpeedLCD.setEnabled(True)
        self.ui.estimatedSpeedLabel.setEnabled(True)
        self.ui.estimatedSpeedLCD.setEnabled(True)
        self.ui.outputPlot.setEnabled(True)
        self.ui.outputPlotLabel.setEnabled(True)
        self.ui.inputPlot.setEnabled(True)
        self.ui.inputPlotLabel.setEnabled(True)
        self.ui.rightPlot.setEnabled(True)
        self.ui.rightPlotLabel.setEnabled(True)
        self.ui.attack1RadioButton.setEnabled(True)
        self.ui.attack2RadioButton.setEnabled(True)
        self.ui.attack3RadioButton.setEnabled(True)
        self.ui.saveInputPlotButton.setEnabled(True)
        self.ui.saveOutputPlotButton.setEnabled(True)
        self.ui.saveRightPlotButton.setEnabled(True)
        self.zeroData()
        
    def disableAllElements(self):
        self.cc(False)
        self.rc(False)
        self.attack(False)

#         for widget in self.mainWidgets:
#             widget.setEnabled(False)

        self.ui.ccButton.setEnabled(False)
        self.ui.rcButton.setEnabled(False)
        self.ui.attackButton.setEnabled(False)
        self.ui.actualLabel.setEnabled(False)
        self.ui.estimatedLabel.setEnabled(False)
        self.ui.desiredSpeedLabel.setEnabled(False)
        self.ui.desiredSpeedEdit.setEnabled(False)
        self.ui.setSpeedButton.setEnabled(False)
        self.ui.kpLabel.setEnabled(False)
        self.ui.kpEdit.setEnabled(False)
        self.ui.setKPButton.setEnabled(False)
        self.ui.kiLabel.setEnabled(False)
        self.ui.kiEdit.setEnabled(False)
        self.ui.setKIButton.setEnabled(False)
        self.ui.autotrimLabel.setEnabled(False)
        self.ui.autotrimEdit.setEnabled(False)
        self.ui.setAutotrimButton.setEnabled(False)
        self.ui.trimLabel.setEnabled(False)
        self.ui.trimValueLabel.setEnabled(False)
        self.ui.setTrimLeftButton.setEnabled(False)
        self.ui.setTrimRightButton.setEnabled(False)
        self.ui.actualSpeedLabel.setEnabled(False)
        self.ui.actualSpeedLCD.setEnabled(False)
        self.ui.estimatedSpeedLabel.setEnabled(False)
        self.ui.estimatedSpeedLCD.setEnabled(False)
        self.ui.outputPlot.setEnabled(False)
        self.ui.outputPlotLabel.setEnabled(False)
        self.ui.inputPlot.setEnabled(False)
        self.ui.inputPlotLabel.setEnabled(False)
        self.ui.rightPlot.setEnabled(False)
        self.ui.rightPlotLabel.setEnabled(False)
        self.ui.attack1RadioButton.setEnabled(False)
        self.ui.attack2RadioButton.setEnabled(False)
        self.ui.attack3RadioButton.setEnabled(False)
        self.ui.saveInputPlotButton.setEnabled(False)
        self.ui.saveOutputPlotButton.setEnabled(False)
        self.ui.saveRightPlotButton.setEnabled(False)

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
            self.rc(False)
            self.attack(False)
            res = self.remote.stopCC()           
        self.ui.ccButton.setChecked(res)

    def rc(self, checked):
        if checked:
            try:
                self.run_rc_pub.publish(Int32(1))
            except:
                self.ui.rcButton.setChecked(False)
        else:
            try:
                self.run_rc_pub.publish(Int32(0))
            except:
                self.ui.rcButton.setChecked(True)
        self.ui.rcButton.setChecked(checked)

    def attack(self, checked):
        if checked:
            try:
            	mode = 0
            	if self.ui.attack1RadioButton.isChecked:
            		mode = 1
            	elif self.ui.attack2RadioButton.isChecked:
            		mode = 2
            	elif self.ui.attack3RadioButton.isChecked:
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
        self.ui.attackButton.setChecked(checked)
        if checked:
            self.toggleWidgetColor(self.ui.actualLabel, "red")
        else:
            self.toggleWidgetColor(self.ui.actualLabel, "green")

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
        return unicode(QtGui.QFileDialog.getSaveFileName(self, 'Save file', '', file_choices))

    def draw_inputPlot(self):
        """ Redraws the input plot
        """
        # clear the axes and redraw the plot anew
        #
        self.inAxes.clear()
        self.inAxes.grid(True)
        self.inAxes.set_ybound(0, self.y_top)
        self.inAxes.set_autoscaley_on(False)
        self.inAxes.set_xlabel('time')
        self.inAxes.set_ylabel('speed')
        self.inAxes.set_title('Input')
        self.inAxes.plot(self.in_Base)
        self.inAxes.plot(self.in_Ref)
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
        self.outAxes.set_autoscaley_on(False)
        self.outAxes.set_xlabel('time')
        self.outAxes.set_ylabel('speed')
        self.outAxes.set_title('Output')
        self.outAxes.plot(self.out_EncL)
        self.outAxes.plot(self.out_EncR)
        self.outAxes.plot(self.out_GPS)
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
        self.rightAxes.set_xlabel('time')
        self.rightAxes.set_ylabel('speed')
        self.rightAxes.set_title('Odometry')
        self.rightAxes.plot(self.out_Odom)
        self.rightAxes.plot(self.in_Ref)
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

        #rospy.signal_shutdown("Turning off ROSPy")

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
        
    def captureEncL(self, msg):
        self.out_EncL.append(msg.twist.linear.x)
        self.draw_outputPlot()
        
    def captureEncR(self, msg):
        self.out_EncR.append(msg.twist.linear.x)
        
    def captureGPS(self, msg):
        self.out_GPS.append(msg.twist.linear.x)

def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    app.aboutToQuit.connect(h.fileQuit)
    app.lastWindowClosed.connect(app.quit)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


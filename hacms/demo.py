import sys, string
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from PySide import QtGui
import matplotlib
matplotlib.use('Qt4Agg')
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
        self.ui.actionQuit.triggered.connect(self.close)
        self.ui.landsharkButton.toggled.connect(self.landshark)
        self.ui.ccButton.toggled.connect(self.cc)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.ui.setSpeedButton.triggered.connect(self.setLandsharkSpeed)
        self.remote = Remote(self.ui.console)

    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
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
            self.disableAllElements()

        self.ui.landsharkButton.setChecked(res)

    def cc(self, checked):
        self.ui.ccButton.setChecked(self.run_cc_pub.publish(Bool(True)) if checked else self.run_cc_pub.publish(Bool(False)))

    def rc(self, checked):
        self.ui.rcButton.setChecked(self.landshark_comm() if checked else False)
        #self.ui.rcButton.setChecked(self.remote.startRC() if checked else self.remote.stopRC())

    def attack(self, checked):
        self.ui.attackButton.setChecked(self.run_attack_pub.publish(Bool(True)) if checked else self.run_attack_pub.publish(Bool(False)))

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

    def setLandsharkSpeed(self):
        odom = Odometry()
        odom.msg.twist.twist.linear.x = float(self.ui.desiredSpeedEdit.getText())
        self.desired_speed_pub.publish(odom)

    def landshark_comm(self):
        # Initialize ROS node
        rospy.init_node('landshark_demo', anonymous=True)

        # Subscribe to HACMS Demo topics
        rospy.Subscriber("/landshark_demo/odom", Odometry, self.updateActualSpeedLCD)
        rospy.Subscriber("/landshark_demo/gps_velocity", TwistStamped, self.updateEstimatedSpeedLCD)
        rospy.Subscriber("/landshark/odom", Odometry, self.updateOutputPlot, {"odom": True})

        self.desired_speed_pub = rospy.Publisher('/landshark_demo/desired_speed', Odometry)
        self.run_cc_pub = rospy.Publisher('/landshark_demo/run_cc', Bool)
        self.run_rc_pub = rospy.Publisher('/landshark_demo/run_rc', Bool)
        self.run_attack_pub = rospy.Publisher('/landshark_demo/run_attack', Bool)

        #TODO: stop subscribers just as the GUI is closed (to prevent bad callback)

        return True


def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


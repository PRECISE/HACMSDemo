import sys, string
import rospy
from std_msgs.msg import String
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
        #self.ui.setSpeedButton.triggered.connect(self.setLandsharkSpeed)
        self.remote = Remote(self.ui.console)

    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")

    def landshark(self, checked):
        if checked:
            res = self.remote.startLandshark()
            #self.landshark_listener()
        else:
            self.rc(False)
            self.attack(False)
            res = self.remote.stopLandshark()

        self.ui.landsharkButton.setChecked(res)
        
    def cc(self, checked):
        #self.ui.ccButton.setChecked(self.remote.startCC() if checked else self.remote.stopCC())
        return

    def rc(self, checked):
        self.ui.rcButton.setChecked(self.landshark_listener() if checked else False)
        #self.ui.rcButton.setChecked(self.remote.startRC() if checked else self.remote.stopRC())

    def attack(self, checked):
        self.ui.attackButton.setChecked(self.remote.startAttack() if checked else self.remote.stopAttack())

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

    def updateOutputPlot(self, msg, args): #encL=False, encR=False, gps=False, odom=False):
        if encL:
            return
        if encR:
            return
        if gps:
            return
        if odom:
            self.ui.estimatedSpeedLCD.display(msg.twist.twist.linear.x)
        
    def landshark_listener(self):
        # Initialize ROS node
        rospy.init_node('demo_listener', anonymous=True)

        # Subscribe to HACMS Demo topics
        rospy.Subscriber("/landshark_demo/odom", Odometry, self.updateActualSpeedLCD)
        rospy.Subscriber("/landshark_demo/gps_velocity", TwistStamped, self.updateEstimatedSpeedLCD)
        rospy.Subscriber("/landshark/odom", Odometry, self.updateOutputPlot, (False, False, False, True))
        
        #TODO: stop subscribers just as the GUI is closed (to prevent bad callback)

        return True

    def landshark_desired_speed_publisher(self):
        pub = rospy.Publisher('demo_desired_speed', String)
        r = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():
            pub.publish(str)
            r.sleep()


def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


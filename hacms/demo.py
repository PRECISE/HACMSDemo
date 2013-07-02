import sys, string
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from PySide import QtGui

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
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.remote = Remote(self.ui.console)
    
    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")
                
    def landshark(self, checked):
        if checked:
            res = self.remote.startLandshark()
            #self.hacms_listener()
        else:
            self.rc(False)
            self.attack(False)
            res = self.remote.stopLandshark()
            
        self.ui.landsharkButton.setChecked(res)
        
    def rc(self, checked):
        self.ui.rcButton.setChecked(self.hacms_listener() if checked else False)
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
        
    def updateActualSpeedLCD(self, twistMsg):
        self.ui.actualSpeedLCD.display(twistMsg.linear.x)
        #self.ui.console.appendPlainText('*** Actual Speed: ' + str(twistMsg.linear.x))
        
    def rosTest(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        self.ui.console.appendPlainText('*** ROS TEST: ' + data.data)

    def hacms_listener(self):
        # Initialize ROS node
        rospy.init_node('demo_listener', anonymous=True)

        # Subscribe to HACMS Demo topics
        #rospy.Subscriber("demo_ui", String, window.rosTest)
        rospy.Subscriber("/landshark_control/odom", Twist, self.updateActualSpeedLCD)
        #rospy.Subscriber("/landshark_control/imu", Twist, window.updateActualSpeedLCD)
        
        return True
    

def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
    
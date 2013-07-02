import sys, string, threading
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
        #self.ui.actionQuit.triggered.connect(self.app.quit)
        self.ui.rosButton.toggled.connect(self.ros)
        self.ui.rcButton.toggled.connect(self.rc)
        self.ui.attackButton.toggled.connect(self.attack)
        self.remote = Remote(self.ui.console)
    
    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")
                
    def ros(self, checked):
        if checked:
            res = self.remote.startROS()
        else:
            self.rc(False)
            self.attack(False)
            res = self.remote.stopROS()
            
        self.ui.rosButton.setChecked(res)
        
    def rc(self, checked):
        self.ui.rcButton.setChecked(self.remote.startRC() if checked else self.remote.stopRC())
        
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
        
    def rosListen(data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        print data.data


class HACMSListener(threading.Thread):
    def __init__(self, window):
        self.window = window
        threading.Thread.__init__(self)
        
    def run(self):
        # in ROS, nodes are unique named. If two nodes with the same
        # node are launched, the previous one is kicked off. The 
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'talker' node so that multiple talkers can
        # run simultaenously.
        rospy.init_node('demo_listener', anonymous=True)

        # Subscribe to HACMS Demo topics
        rospy.Subscriber("demo_ui", String, self.window.rosListen)
        #rospy.Subscriber("/landshark_control/odom", Twist, window.updateActualSpeedLCD)
    

def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    HACMSListener(h).start()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
    
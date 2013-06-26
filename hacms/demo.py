import sys, string
#import rospy
#from std_msgs.msg import String
#from geometry_msgs.msg import Twist
from PySide import QtGui

# HACMS Python modules
from remote import Remote
import ui

class HACMSDemoWindow(QtGui.QMainWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.startRobotButton.clicked.connect(self.startRobot)
        self.ui.startRCButton.clicked.connect(self.startRC)
        self.ui.attackButton.clicked.connect(self.attack)
        self.remote = Remote()
    
    def about(self):
        QtGui.QMessageBox.about(self, "About HACMS Demo",
                "The <b>HACMS Demo</b> application displays the current ROS telemetry "
                "information.")
                
    def startRobot(self):
        #print "startRobot"
        #remote.connect()
        #remote.startROS()
        self.toggleWidgetColor(self.ui.expectedLabel)
        
    def startRC(self):
        #print "startRC"
        #remote.connect()
        #remote.startRC()
        self.toggleWidgetColor(self.ui.actualLabel)
        
    def attack(self):
        print "attack"
        #remote.connect()
        #remote.attack()
    
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
        
def ros_listen_test(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print data.data
        
def hacms_listener(window):
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('demo_listener', anonymous=True)

    # Subscribe to HACMS Demo topics
    rospy.Subscriber("demo_ui", String, ros_listen_test)
    #rospy.Subscriber("/landshark_control/odom", Twist, window.updateActualSpeedLCD)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
    app = QtGui.QApplication(sys.argv)
    h = HACMSDemoWindow()
    h.show()
    #hacms_listener(h)
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
    
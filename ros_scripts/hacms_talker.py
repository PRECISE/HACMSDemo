#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'demo_ui' topic

import rospy
from std_msgs.msg import String
#from geometry_msgs.msg import Twist

def hacms_talker():
    pub = rospy.Publisher('demo_ui', String)
    rospy.init_node('demo_talker', anonymous=True)
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        #pub.publish(twistMsg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        hacms_talker()
    except rospy.ROSInterruptException: 
    	pass

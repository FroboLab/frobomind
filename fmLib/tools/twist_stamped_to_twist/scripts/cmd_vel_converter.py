#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Bool

class CmdVelConverter():
    """
        Converter for using FroboMind with stage. 
        Takes TwistStamped message from /fmSignals/cmd_vel and parses as Twist message on /cmd_vel
    """
    def __init__(self):
        # Init node
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist)
        self.twist_sub = rospy.Subscriber("/fmSignals/cmd_vel", TwistStamped, self.onTwist )
        self.deadman_sub = rospy.Subscriber("/fmSignals/deadman", Bool, self.onDeadman )
        self.twist = Twist()
        self.deadman = False
        
 
    def onTwist(self,msg):
        if self.deadman :
            self.twist = msg.twist
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0        
        self.twist_pub.publish(self.twist)
        
    def onDeadman(self,msg):
        self.deadman = msg.data


if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    node = CmdVelConverter()
    rospy.spin()
    



    
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist

LIN_Z_SPD = 0.2

class CtrlAlti():

    def __init__(self):
    
        rospy.init_node('sub_altitude')
        
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged',\
                         Ardrone3PilotingStateAltitudeChanged,
                         self.get_alti 
                        )
        #self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        #self.tw  = Twist()
        self.alti = 0.0
    
    def get_alti(self, msg):
        self.alti = msg.altitude
        #print("altitude = %s(m)" %(self.alti))
        

if __name__ == '__main__': 
    try:
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        ca = CtrlAlti()
        tw = Twist()
        target = float(input("input target altitude: "))
        
        if target - ca.alti > 0:
            tw.linear.z =  LIN_Z_SPD
            while(target > ca.alti):
                pub.publish(tw)
            tw.linear.z =  0.0
            pub.publish(tw)
        else:
            tw.linear.z = -LIN_Z_SPD
            while(target < ca.alti):
                pub.publish(tw)
            tw.linear.z =  0.0
            pub.publish(tw)
        
        rospy.spin()
 
    except rospy.ROSInterruptException:
        pass

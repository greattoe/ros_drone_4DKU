#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
 


class SubAlti():

    def __init__(self):
    
        rospy.init_node('sub_altitude')
        
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged',\
                         Ardrone3PilotingStateAltitudeChanged,
                         self.get_alti 
                        )
        self.alti = 0.0
    
    def get_alti(self, msg):
        self.alti = msg.altitude
        print("altitude = %s(m)" %(self.alti))
        

if __name__ == '__main__': 
    try:
        SubAlti()
        rospy.spin()
 
    except rospy.ROSInterruptException:
        pass

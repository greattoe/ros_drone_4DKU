#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from getchar import GetChar    

MAX_LIN_SPD =  0.22
MIN_LIN_SPD = -0.22

MAX_ANG_SPD =  2.84
MIN_ANG_SPD = -2.84

LIN_STP = 0.1
ANG_STP = 0.2

rospy.init_node('remote_turtlebot')
pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
tw   = Twist()
kb   = GetChar()

msg  = """
---------------------------------------
              (forward)
                 'w'

  (left)'a'      's'       'd'(right)
              (backward)
---------------------------------------
type 'Q' for quit program...
---------------------------------------
"""

print(msg)

while not rospy.is_shutdown():
    
    ch = kb.getch()
    
    if   ch == 'w':
        if  tw.linear.x +  LIN_STP <= MAX_LIN_SPD:
            tw.linear.x =  tw.linear.x + LIN_STP
        else:
            tw.linear.x =  MAX_LIN_SPD
            
    elif ch == 's':
        if  tw.linear.x - LIN_STP >= MIN_LIN_SPD:
            tw.linear.x =  tw.linear.x - LIN_STP
        else:
            tw.linear.x =  MIN_LIN_SPD
            
    elif ch == 'a':
        if  tw.angular.z + ANG_STP <= MAX_ANG_SPD:
            tw.angular.z = tw.angular.z + ANG_STP
        else:
            tw.angular.z = MAX_ANG_SPD
            
    elif ch == 'd':
        if  tw.angular.z - ANG_STP >= MIN_ANG_SPD:
            tw.angular.z = tw.angular.z - ANG_STP
        else:
            tw.angular.z = MIN_ANG_SPD
            
    elif ch == ' ':
        tw.linear.x  = tw.angular.z = 0
            
    elif ch == 'Q':
        break
        
    else:
        pass
        
    pub.publish(tw)
    print("linear_speed: %s, \tangular_speed: %s" %(tw.linear.x, tw.angular.z))
    
    pub.publish(tw)

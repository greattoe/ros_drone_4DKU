#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from getchar import GetChar    

rospy.init_node('remote_turtle')
pub  = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
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
        tw.linear.x =  2.0;    tw.angular.z =  0.0
    elif ch == 's':
        tw.linear.x = -2.0;    tw.angular.z =  0.0
    elif ch == 'a'  :
        tw.linear.x =  0.0;    tw.angular.z =  2.0
    elif ch == 'd':
        tw.linear.x =  0.0;    tw.angular.z = -2.0
    elif ch == 'Q':
        break
    else:
        pass
    
    pub.publish(tw)

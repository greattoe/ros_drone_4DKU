#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from std_msgs.msg import Bool
from bebop_move import Bebop2Move

LIN_SPD    =   0.55

if __name__ == '__main__':
    
    rospy.init_node('drop_load', anonymous = True)    
    drop = rospy.Publisher('/dropper', Bool, queue_size=1)
    down = Bool()
    bb2  = Bebop2Move()
    
    while rospy.get_param('mission_1_finished') is False:
        pass
       
    for i in range(10):
        print("count down: %s" %(9-i))
        rospy.sleep(1.0)
    
    print "descend to height for release load"
    bb2.move_z(-2.5, 0.05)
    
    print "#### release load ####"
    
    down = True
    drop.publish(down)
    
    rospy.set_param('mission_2_finished', True)
    
    sys.exit(1)

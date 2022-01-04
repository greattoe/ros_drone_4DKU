#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from bebop2_pkg.msg import Pos_XYZ_th
from geometry_msgs.msg import Twist

LIN_SPD = 0.2

class OdomPose():

    def __init__(self):
    
        rospy.init_node('sub_altitude')
        
        rospy.Subscriber('/bebop_odom_pose', Pos_XYZ_th, self.get_odom_pose)
        #self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        #self.tw  = Twist()
        self.odom_pose_now = 0.0
        self.odom_pose_org = 0.0
    
    def get_odom_pose(self, msg):
        self.odom_pose_now = msg
    
    def get_org(self):
        self.odom_pose_org = self.odom_pose_now
        

if __name__ == '__main__': 
    try:
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        op = OdomPose()
        tw = Twist()
        target = float(input("input distance to move y direction: ")) * 0.67
        
        if target >= 0:
            tw.linear.y =  LIN_SPD
            op.get_org()
            while(op.odom_pose_now.y < op.odom_pose_org.y + target):
                print("%s / %s" %(op.odom_pose_now.x, op.odom_pose_org.x + target))
                pub.publish(tw)
            tw.linear.y =  0.0
            pub.publish(tw)
        else:
            tw.linear.y = -LIN_SPD
            op.get_org()
            while(op.odom_pose_now.y > op.odom_pose_org.y + target):
                pub.publish(tw)
            tw.linear.y =  0.0
            pub.publish(tw)
        
        rospy.spin()
 
    except rospy.ROSInterruptException:
        pass

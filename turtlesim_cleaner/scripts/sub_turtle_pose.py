#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose

class TurtlePose:

    def __init__(self):
        rospy.init_node('get_turtle_pose', anonymous=True)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):
        self.pose   = msg
        
        print "x=%s\ty=%s\ttheta=%s" %(self.pose.x, self.pose.y, self.pose.theta)

if __name__ == '__main__':
    try:
        TurtlePose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
from turtlesim_cleaner.srv import AngleDistance, AngleDistanceResponse
import rospy
from geometry_msgs.msg import Twist
from math import radians

LIN_X = ANG_Z = 1.5

def svc_cb(req):
    res_rot = rotate(radians(req.angle))
    res_mov = move(req.distance)
    if res_rot and res_mov is True:
        print "turtle is arrived destination!"
    return AngleDistanceResponse(res_rot and res_mov)

def turtlesim_svc_svr():
    rospy.init_node('turtlesim_svc_node')
    svc = rospy.Service('turtlesim_svc', AngleDistance, svc_cb)
    print "turtle1 ready to move~"
    rospy.spin()

def rotate(angle):
    p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    t = Twist()
    speed  = ANG_Z

    if angle < 0:
        t.angular.z = -speed
    else:
        t.angular.z =  speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < abs(angle)):
        p.publish(t)
        t1 = rospy.Time.now().to_sec()
        current_angle = speed * (t1 - t0)
    
    t.angular.z = 0;    p.publish(t);   print "end rotate"
    return True

def move(distance):
    p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    t = Twist()
    speed = LIN_X    

    if distance < 0:
        t.linear.x = -speed
    else:
        t.linear.x =  speed

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < abs(distance)):
        p.publish(t)
        t1 = rospy.Time.now().to_sec()
        current_distance= speed * (t1 - t0)
    
    t.linear.x = 0;     p.publish(t);   rospy.sleep(3); print "end move"
    return True


if __name__ == "__main__":
    turtlesim_svc_svr()


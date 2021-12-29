#!/usr/bin/env python
import sys
import rospy
from turtlesim_cleaner.srv import AngleDistance

def move_turtle_client(angle, distance):
    rospy.wait_for_service('turtlesim_svc')

    try:
        svc = rospy.ServiceProxy('turtlesim_svc', AngleDistance)
        res = svc(angle, distance)
        return res.complete
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [angle] [distance]" %sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        angle = float(sys.argv[1])
        distance = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    print "Requesting rotate %s(deg) & move %s(m)"%(angle, distance)
    print "Request is %s" \
    %("complete!" if move_turtle_client(angle, distance) else "incomplete")
    

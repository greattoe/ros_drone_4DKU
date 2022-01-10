#!/usr/bin/env python

import sys
import rospy
from serial import Serial
from std_msgs.msg import Bool

sp = Serial('/dev/ttyUSB0', 9600, timeout=1)

class Drop:

  def __init__(self):
    rospy.Subscriber('/dropper', Bool, self.dropper_ctrl) 

  def dropper_ctrl(self, msg):
    if msg.data is True:
       sp.write('1');  print("release!")

if __name__ == '__main__':
  try:
    rospy.init_node("dropper_ctrl", anonymous=False)
    Drop()
    rospy.spin()
        
  except rospy.ROSInterruptException:  pass

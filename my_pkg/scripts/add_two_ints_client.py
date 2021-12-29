#!/usr/bin/env python
# -*- coding: utf-8 -*-
# my_pkg 폴더의 하위폴더 srv의 모든 서비스 
import rospy, sys
from my_pkg.srv import *

def add_two_ints_client(x, y):	# 
    # 서비스를 호출하는 클라이언트 코드는 간단한 편이다.init_node()를 호출할 필요도없다.
    # 시작 시  wait_for_service()를 호출해야 하는데,이 함수는 편리하게도'add_two_ints'
    # 서비스가 사용가능할 때까지 block 시킨다.
    rospy.wait_for_service('add_two_ints')
    try:
        # 서비스호출을 위한 handle 생성
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # 생성한 handle을 이용한 서비스 호출
        resp1 = add_two_ints(x, y)
        return resp1.sum
    # 서비스 호출이 실패할 경우의 예외처리
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:	# 서비스에 필요한 인수의 개 수가 맞게 입력된 경우
        x = int(sys.argv[1])	# x에 첫번째 인수를,
        y = int(sys.argv[2])	# y에 두번째 인수를 치환.
    else:			# 인수의 개 수가 틀린 경우
        print usage()		# usage() 함수를 호출하고,
        sys.exit(1)		# 프로그램 종료.
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))

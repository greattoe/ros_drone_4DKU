#!/usr/bin/env python
# -*- coding: utf-8 -*-
# my_pkg 폴더의 하위폴더 srv 에서 AddTwoInts.srv 서비스와 그 응답서비스 import
from my_pkg.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def svc_cb(req):    # 클라이언트 노드의 서비스 요청 발생시 호출될 콜백함수 svc_cb() 정의
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server') # 노드 초기화 (노드명:'add_two_ints_server')
    # 서비스명:'add_two_ints',서비스타입:AddTwoInts.srv,콜백함수:svc_cb()인 서비스 s 선언
    s = rospy.Service('add_two_ints', AddTwoInts, svc_cb)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

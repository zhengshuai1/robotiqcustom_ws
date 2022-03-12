#!/usr/bin/env python
from __future__ import print_function

import rospy

# import sys
# sys.path.append("..")
# from utils import gripper_control
# from utils.gripper_control import GripperController

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import time

def main():
    rospy.init_node("gripper_client_test_node")
    group_name = 'left_arm'
    service_name = '/' + group_name + '/gripper_control'
    try:
        rospy.wait_for_service(service_name, 1)
        client = rospy.ServiceProxy(service_name, SetBool)
    except rospy.ROSException:
        rospy.logwarn('Service ' + service_name + ' not available')
    req = SetBoolRequest()
    req.data = True
    tic = time.time()
    resp = client(req)
    print('time is', time.time()-tic)
    rospy.sleep(2)
    tic = time.time()
    req.data = False
    resp = client(req)
    print('time is', time.time() - tic)



if __name__ == "__main__":
    main()
#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    ns = rospy.get_namespace()
    node_name = rospy.get_name()
    srv_get_left_arm_running_state = rospy.Service('get_running_state', SetBool,
                                                         get_left_arm_running_state_handle)
    print('ns', ns, ns+'joint_states','node name is', node_name)

    while not rospy.is_shutdown():
        rospy.sleep(1)
        rospy.spin()

def get_left_arm_running_state_handle():
    pass

if __name__ == "__main__":
    main()
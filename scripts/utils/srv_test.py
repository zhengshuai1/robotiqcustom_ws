import rospy
import actionlib


def motion_test(self, goals, goal_type=0):
    """goals =[left arm, right arm]"""
    service_name = "execute_all_joint_poses"
    group_name = 'dual_arm'
    try:
        rospy.wait_for_service(service_name, 1)
        client = rospy.ServiceProxy(service_name, ExecuteAllJointPoses)
    except rospy.ROSException:
        rospy.logwarn('Service ' + service_name + ' not available')
        return None
    req = ExecuteAllJointPosesRequest()
    req.group_name = group_name
    req.goals = ros_utils.to_posearray_msg(goals)
    req.goal_type = goal_type
    resp = client(req)
    if resp.result_status == resp.FAILED:
        rospy.logerr('execute both joint pose failed')
        return False
    return True

def main():

    rospy.init_node("nachi_test")
    arm_grasp.run_ee_sim()


if __name__ == "__main__":
    main()


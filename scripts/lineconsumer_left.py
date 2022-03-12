#!/usr/bin/env python

import rospy
from geometry_msg import PoseStamped

class LineConsumer(object):
    def __init__(self):
        super().__init__()
        self.load_params()
        self.name = rospy.get_name()
#         self.conveyor_speed = []
    def load_params(self)
        try:
            self.prefix = rospy.get_param('/line_manager/ros_prefix')
            self.conveyor_speed = rospy.get_param('/conveyor_speed')
            self.cam_dist = rospy.get_param(f'/line_consumers/{self.name}/cam_dist')
            self.classes = rospy.get_param(f'/line_consumers/{self.name}/classes')
            self.queue_size = rospy.get_param(f'/line_consumers/{self.name}/queue_size')
            self.time_margin = rospy.get_param(f'/line_consumers/{self.name}/time_margin')
            self.tot_tm = self.cam_dist / self.conveyor_speed - self.time_pick - self.time_margin
        except KeyError as e:
            rospy.logwarn(f"Param {e} not found. Either no param file was loaded or your param file is incomplete")
            rospy.logwarn("Using default values")
            return

    def do_picking(self, req):
        if rospy.Time.now() - req.header.stamp > self.tot_tm :
            return
        else:
            move_wait()
            while rospy.Time.now() - req.header.stamp > self.cam_dist / self.conveyor_speed - self.time_pick:
                rospy.sleep(0.1)
            pick()
            move_rest()
        return

    def move_rest(self):
        # TODO Move arm to wait pose

    def move_wait(self, pose):
        # TODO Move arm to wait pose

    def pick(self):
        # TODO Pick and dispose of item

    def start(self):
        rospy.init_node("robot")
        rospy.Subscriber(f"{self.prefix}/{self.name}", PosedStamped, self.do_picking)
        self.move_rest()
        rospy.spin()

if __name__="__main__":
    c = LineConsumer()
    c.start()
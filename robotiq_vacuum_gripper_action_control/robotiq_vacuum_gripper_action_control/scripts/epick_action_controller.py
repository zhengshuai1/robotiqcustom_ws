#!/usr/bin/env python
import rospy, os
import numpy as np
# from sensor_msgs.msg import JointState
# Actionlib
from actionlib import SimpleActionServer
from robotiq_msgs.msg import CModelCommandAction, CModelCommandFeedback, CModelCommandResult

from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippers_robot_input as CModelStatus
from robotiq_vacuum_grippers_control.msg import RobotiqVacuumGrippers_robot_output as CModelCommand

def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)


class CModelActionController(object):
  def __init__(self, activate=True):
    self._name = rospy.get_param("~gripper_name")
    # Read configuration parameters
    self._fb_rate = read_parameter('vacuum_gripper/publish_rate', 100.0)
    self._min_gap = read_parameter('vacuum_gripper/min_gap', 0.0)
    self._max_gap = read_parameter('vacuum_gripper/max_gap', 255.0)
    self._min_speed = read_parameter('vacuum_gripper/min_speed', 0)
    self._max_speed = read_parameter('vacuum_gripper/max_speed', 25.5)
    self._min_force = read_parameter('vacuum_gripper/min_force', 10)
    self._max_force = read_parameter('vacuum_gripper/max_force', 78)

    # Configure and start the action server
    self._status = CModelStatus()

    self._server = SimpleActionServer(self._name, CModelCommandAction, execute_cb=self._execute_cb, auto_start=False)
    rospy.Subscriber('input', CModelStatus, self._status_cb, queue_size=1)
    self._cmd_pub = rospy.Publisher('output', CModelCommand, queue_size=1)
    working = True
    # if self._status.gFLT:
    #   self._reset()
    if activate and not self._ready():
      # rospy.sleep(0.2)
      working = self._activate()
    # if not working:
    #   return
    self._server.start()
    rospy.loginfo('%s: Started' % self._name)
    rospy.logdebug('%s: Started' % self._name)

  def _preempt(self):
    self._stop()
    rospy.loginfo('%s: Preempted' % self._name)
    self._server.set_preempted()

  def _status_cb(self, msg):
    self._status = msg

  def _execute_cb(self, goal):
    success = True
    # Check that the gripper is active. If not, activate it.
    if not self._ready():
      if not self._activate():
        rospy.logwarn('%s could not accept goal because the gripper is not yet active' % self._name)
        return
    # check that preempt has not been requested by the client
    if self._server.is_preempt_requested():
      self._preempt()
      return
    # Clip the goal

    position = np.clip(goal.position, self._min_gap, self._max_gap)
    velocity = np.clip(goal.velocity, self._min_speed, self._max_speed)
    force = np.clip(goal.force, self._min_force, self._max_force)
    mod = int(goal.mod)
    self._command = CModelCommand()
    self._command.rPR = int(position)
    self._command.rMOD = mod
    # Send the goal to the gripper and feedback to the action client
    rate = rospy.Rate(self._fb_rate)
    rospy.loginfo('%s: Moving gripper to position: %.3f ' % (self._name, position))
    feedback = CModelCommandFeedback()
    while not self._reached_goal():
      self._goto_position(mod, position, velocity, force)
      if rospy.is_shutdown() or self._server.is_preempt_requested():
        self._preempt()
        return
      feedback.position = self._status.gPR
      feedback.stalled = self._stalled()
      feedback.reached_goal = self._reached_goal()
      self._server.publish_feedback(feedback)
      # rate.sleep()
      rospy.sleep(0.01)
      # if self._stalled():rosl
      #   break
    rospy.loginfo('%s: Succeeded' % self._name)
    result = CModelCommandResult()
    result.position = self._status.gPR
    result.stalled = self._stalled()
    result.reached_goal = self._reached_goal()
    self._server.set_succeeded(result)

  def _activate(self, timeout=1.0):
    command = CModelCommand()
    command.rMOD = 0
    command.rACT = 1
    command.rGTO = 1
    command.rPR = 255
    # command.rSP  = 255
    # command.rFR  = 150
    # start_time = rospy.get_time()
    # while not self._ready():
    #   if rospy.is_shutdown():
    #     self._preempt()
    #     return False
    #   if rospy.get_time() - start_time > timeout:
    #     rospy.logwarn('Failed to activated gripper in ns [%s]' % (self._name))
    #     return False
    #   self._cmd_pub.publish(command)
    #   rospy.sleep(0.1)
    # rospy.loginfo('Successfully activated gripper in ns [%s]' % (self._name))
    # return True

    if rospy.is_shutdown():
      self._preempt()
      return False
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout:
      self._cmd_pub.publish(command)
      rospy.sleep(0.5)
    # rospy.loginfo('Successfully activated gripper in ns [%s]' % (self._name))
    return True

  def _reset(self, timeout=1.0):
    command = CModelCommand()
    command.rMOD = 0
    command.rACT = 0
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout:
      self._cmd_pub.publish(command)
      rospy.sleep(0.2)
    rospy.loginfo('Finish reset gripper')


  def _goto_position(self, mod, pos, vel, force):
    """
    Goto position with desired force and velocity
    @type  pos: float
    @param pos: Gripper width in meters
    @type  vel: float
    @param vel: Gripper speed in m/s
    @type  force: float
    @param force: Gripper force in N
    """
    command = CModelCommand()
    command.rACT = 1
    command.rGTO = 1
    if mod == 0:
      command.rPR = int(np.clip(pos, 0, 255))
      command.rMOD = 0
    elif mod == 1:
      command.rMOD = 1
      command.rPR = int(np.clip(pos, 0, 255))
      # command.rSP = int(np.clip(vel, 0, 255))
      # command.rFR = int(np.clip(force, 0, 80))
    else:
      rospy.logerr('unknown number for gripper mode')
      return
    self._cmd_pub.publish(command)

  def _moving(self):
    return self._status.gGTO == 1 and self._status.gOBJ == 0

  def _reached_goal(self):
    # for vacuum gripper
    # return self._status.gOBJ == 1 or self._status.gOBJ == 2
    ok = self._status.gGTO and self._status.gOBJ and self._status.gPR == self._command.rPR
    return ok

  def _ready(self):
    return self._status.gSTA == 3 and self._status.gACT == 1

  def _errorflag(self):
    return self._status.gFLT

  def _stalled(self):
    return self._status.gOBJ == 1 or self._status.gOBJ == 2

  def _stop(self):
    command = CModelCommand()
    command.rACT = 1
    command.rGTO = 0
    self._cmd_pub.publish(command)
    rospy.logdebug('Stopping gripper in ns [%s]' % (self._name))


if __name__ == '__main__':
  node_name = 'robotiq_vacuum_gripper_action_server'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  cmodel_server = CModelActionController()
  rospy.spin()
  rospy.loginfo('Shutting down [%s] node' % node_name)

/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"

// To keep the fully qualified names managable

//Anonymous namespaces are file local -> sort of like global static objects
namespace
{
  using namespace robotiq_2f_gripper_action_server;

  /*  This struct is declared for the sole purpose of being used as an exception internally
      to keep the code clean (i.e. no output params). It is caught by the action_server and 
      should not propogate outwards. If you use these functions yourself, beware.
  */
  struct BadArgumentsError {};


  GripperOutput goalToRegisterState(const GripperCommandGoal& goal, const Robotiq2FGripperParams& params)
  {
    GripperOutput result;
    result.rACT = 0x1; // active gripper
    result.rGTO = 0x1; // go to position
    result.rATR = 0x0; // No emergency release
    result.rSP = 255; // Middle ground speed
    
    if (goal.command.position > params.max_gap_ || goal.command.position < params.min_gap_)
    {
      ROS_WARN("Goal gripper gap size is out of range(%f to %f): %f m",
               params.min_gap_, params.max_gap_, goal.command.position);
      throw BadArgumentsError();
    }
    
    if (goal.command.max_effort < params.min_effort_ || goal.command.max_effort > params.max_effort_)
    {
      ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
               params.min_effort_, params.max_effort_, goal.command.max_effort);
      throw BadArgumentsError();
    }

    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.rPR = static_cast<uint8_t>((params.max_gap_ - goal.command.position) / dist_per_tick);
    result.rFR = static_cast<uint8_t>((goal.command.max_effort - params.min_effort_) / eff_per_tick);

    ROS_INFO("Setting goal position register to %hhu", result.rPR);

    return result;
  }

  /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
      of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
      code duplication.
  */
  template<typename T>
  T registerStateToResultT(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    T result;
    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.position = input.gPO * dist_per_tick + params.min_gap_;
    result.effort = input.gCU * eff_per_tick + params.min_effort_;
    result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;
    result.reached_goal = input.gPO == goal_pos;

    return result;
  }

  // Inline api-transformers to avoid confusion when reading the action_server source
  inline
  GripperCommandResult registerStateToResult(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandResult>(input, params, goal_pos);
  }

  inline
  GripperCommandFeedback registerStateToFeedback(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandFeedback>(input, params, goal_pos);
  }

} // end of anon namespace

namespace robotiq_2f_gripper_action_server
{

Robotiq2FGripperActionServer::Robotiq2FGripperActionServer(const std::string& name, const Robotiq2FGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&Robotiq2FGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&Robotiq2FGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1, &Robotiq2FGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);

//  // Check for errors
//  if (current_reg_state_.gFLT)
//  {
//    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
//    while (current_reg_state_.gFLT)
//    {
//      reset();
//      ros::Duration(0.1).sleep();
//    }
//    ROS_INFO(" Successfully clear fault, reset gripper");
//  }
  reset();
  ROS_INFO("reset gripper before active");
//  ros::Duration(0.1).sleep();

  as_.start();
  ROS_INFO(" Successfully start action %s", action_name_.c_str());

//  while ( !(current_reg_state_.gSTA == 0x3 && current_reg_state_.gACT == 0x1) )
//  {
//    // If it hasn't been asked, active it
//    issueActivation();
//    ROS_INFO(" gSTA is  %x, gACT is  %x, %d, %d", current_reg_state_.gSTA, current_reg_state_.gACT,  current_reg_state_.gGTO,
//     current_reg_state_.gFLT);
//    ros::Duration(0.1).sleep();
//  }
//  ROS_INFO(" Successfully activated gripper ");
}

void Robotiq2FGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
//  while (current_reg_state_.gSTA != 0x3 && goal_reg_state_.rACT != 0x1)
//  {
//    // If it hasn't been asked, active it
//    issueActivation();
//    ros::Duration(0.01).sleep();
//  }
  if (current_reg_state_.gSTA != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
//    return;
    issueActivation();
    ros::Duration(0.1).sleep();
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_);
    goal_pub_.publish(goal_reg_state_);
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
  }
}

void Robotiq2FGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void Robotiq2FGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;

  if (!as_.isActive()) return;

    // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
     return;
//    while (current_reg_state_.gFLT)
//    {
//      reset();
//      ros::Duration(0.1).sleep();
//    }
//    reset();
//    ros::Duration(0.1).sleep();
////    issueActivation();
////    ros::Duration(0.1).sleep();
//    ROS_INFO(" Successfully clear fault, reset and activate gripper");
  }
  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gSTA != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gSTA == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend activate
    return;
  }

  // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_,
                                         gripper_params_,
                                         goal_reg_state_.rPR));
  }
  else if (current_reg_state_.gGTO && current_reg_state_.gOBJ && current_reg_state_.gPR == goal_reg_state_.rPR)
  {
    // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're
    // done with a move
    ROS_INFO("%s succeeded", action_name_.c_str());
    as_.setSucceeded(registerStateToResult(current_reg_state_,
                                           gripper_params_,
                                           goal_reg_state_.rPR));
  }
  else
  {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(current_reg_state_,
                                                gripper_params_,
                                                goal_reg_state_.rPR));
  }
}

void Robotiq2FGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  out.rGTO = 0x1; // go to position
//  out.rPR = 0;
  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}

void Robotiq2FGripperActionServer::reset()
{
  ROS_INFO("Resset gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x0;
  // other params should be zero
  goal_reg_state_ = out;
  ros::Duration timeout = ros::Duration(1.0);
  ros::Time start_time = ros::Time::now();
  while(ros::Time::now()-start_time < timeout)
  {
    goal_pub_.publish(out);
    ros::Duration(0.5).sleep();
  }

}
} // end robotiq_2f_gripper_action_server namespace

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionClient");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/pr2_tasks_node/plan", true);

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/pr2_tasks_node/execute", true);


  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  plan.waitForServer(); //will wait for infinite time
  execute.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");
  pr2_motion_tasks_msgs::planGoal planGoal;
  planGoal.objId = "cube_GBTG_2";
  planGoal.action = "pick";
  plan.sendGoal(planGoal);

  //wait for the action to return
  bool finished_before_timeout_pick = plan.waitForResult();
  ROS_INFO_STREAM("Solution have cost : " << plan.getResult()->cost);

  if(finished_before_timeout_pick && (plan.getResult()->error_code == 1))
  {
    ROS_INFO("Sending execute goal !");
    pr2_motion_tasks_msgs::executeGoal executeGoal;
    execute.sendGoal(executeGoal);

    bool finished_before_timeout_execute = execute.waitForResult();
    ROS_INFO_STREAM("Execution finished with status : " << execute.getResult()->error_code);

    planGoal.planGroup = "right_arm";
    planGoal.boxId = "box_C5";
    planGoal.action = "place";
    plan.sendGoal(planGoal);

    plan.waitForResult();
    ROS_INFO_STREAM("Solution have cost : " << plan.getResult()->cost);
    execute.sendGoal(executeGoal);

    execute.waitForResult();
    ROS_INFO_STREAM("Execution finished with status : " << execute.getResult()->error_code);
  }


  ros::waitForShutdown();
  return 0;
}

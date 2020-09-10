#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_mtc/pickAction.h>
#include <pr2_mtc/placeAction.h>
#include <pr2_mtc/moveAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionClient");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  actionlib::SimpleActionClient<pr2_mtc::pickAction> pick("/pr2_tasks_node/pick", true);
  actionlib::SimpleActionClient<pr2_mtc::placeAction> place("/pr2_tasks_node/place", true);
  actionlib::SimpleActionClient<pr2_mtc::moveAction> move("/pr2_tasks_node/move", true);


  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  pick.waitForServer(); //will wait for infinite time
  place.waitForServer(); //will wait for infinite time
  move.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");
  pr2_mtc::pickGoal pickGoal;
  pickGoal.objId = "cube_GBTG_2";
  pickGoal.planOnly = false;
  pick.sendGoal(pickGoal);

  //wait for the action to return
  bool finished_before_timeout_pick = pick.waitForResult();
  ROS_INFO_STREAM("Solution have cost : " << pick.getResult()->cost);

  if(finished_before_timeout_pick)
  {
    ROS_INFO("Sending place goal !");
    pr2_mtc::placeGoal placeGoal;
    placeGoal.objId = "cube_GBTG_2";
    placeGoal.boxId = "box_C5";
    placeGoal.planOnly = false;
    place.sendGoal(placeGoal);
  }


  ros::waitForShutdown();
  return 0;
}

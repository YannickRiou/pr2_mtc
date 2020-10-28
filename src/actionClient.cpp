#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

// Head related include
#include <pr2_controllers_msgs/PointHeadAction.h>

void lookAt(std::string frame_id, double x, double y, double z)
{

    // Our Action interface type, provided as a typedef for convenience
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

  PointHeadClient* point_head_client;

  //Initialize the client for the Action interface to the head controller
  point_head_client = new PointHeadClient("/head_traj_controller/point_head_action", true);

  //wait for head controller action server to come up
  while(!point_head_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }

  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x;
  point.point.y = y;
  point.point.z = z;
  goal.target = point;

  goal.pointing_frame = "head_mount_kinect2_rgb_optical_frame";
  goal.pointing_axis.x = 0;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 1;

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);

  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  //send the goal
  point_head_client->sendGoal(goal);

  //wait for it to get there (abort after 2 secs to prevent getting stuck)
  point_head_client->waitForResult(ros::Duration(2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionClient");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  char ch;

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/pr2_tasks_node/plan", true);

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/pr2_tasks_node/execute", true);

  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  plan.waitForServer(); //will wait for infinite time
  execute.waitForServer(); //will wait for infinite time

  lookAt("box_C5",0,0,0);

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  }

  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;
  
  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
   ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;
  
  if (execute.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
   ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code== 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
   ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    lookAt("box_C3",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.boxId = "box_C3";
    planGoal.action = "place";
    plan.sendGoal(planGoal);
    plan.waitForResult();
 
    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
   ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }

  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;
  
  if (execute.getResult()->error_code == 1)
  {
    lookAt("box_C5",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.boxId = "box_C5";
    planGoal.action = "place";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    plan.sendGoal(planGoal);
    plan.waitForResult();
    
    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
   ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
   std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "drop";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  
  ROS_WARN_STREAM("==============PRESS [ENTER] FOR NEXT STEP==============");
  std::cin >> ch;

  if (execute.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "drop";
    plan.sendGoal(planGoal);
    plan.waitForResult();

    if(plan.getResult()->error_code == 1)
    {
      execute.sendGoal(executeGoal);
      execute.waitForResult();
    }
  }
  }
  }
  }
  }
  }
  }
  }
  }
  }
  }
  



  ros::waitForShutdown();
  return 0;
}

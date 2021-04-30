#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

#include <moveit/move_group_interface/move_group.h>
// Head related include
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <geometry_msgs/Twist.h>

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

void scenario_replace_2(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction>& planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  lookAt("box_B5",0,0,0);

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
     else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }

   lookAt("box_C5",0,0,0); 
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
        else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
  if (executeClient.getResult()->error_code== 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
      else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("box_C5",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.boxId = "box_C5";
    planGoal.action = "place";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
 
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
        else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("box_B5",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.boxId = "box_B5";
    planGoal.action = "place";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
 
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  }
  }
  }
  }
  }
  }
  }
}

void scenario_replace(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction>& planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  lookAt("box_B5",0,0,0);

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
     else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }

   lookAt("box_C5",0,0,0); 
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
        else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
  if (executeClient.getResult()->error_code== 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
      else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("box_C5",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.boxId = "box_C5";
    planGoal.action = "place";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
 
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
        else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("box_B5",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.boxId = "box_B5";
    planGoal.action = "place";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
 
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }

  lookAt("box_C5",0,0,0);
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

  lookAt("box_B5",0,0,0);

  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
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
}

void scenario_multi_drop(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("throw_box_left",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
   if (executeClient.getResult()->error_code == 1)
  {
    lookAt("throw_box_left",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

    if (executeClient.getResult()->error_code== 1)
    {
      lookAt("cube_BBCG",0,0,0);
      planGoal.planGroup = "left_arm";
      planGoal.objId = "cube_BBCG";
      planGoal.action = "pick";
      planClient.sendGoal(planGoal);
      planClient.waitForResult();

      if(planClient.getResult()->error_code == 1)
      {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
      }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

    if (executeClient.getResult()->error_code == 1)
    {
      lookAt("throw_box_left",0,0,0);
      planGoal.planGroup = "left_arm";
      planGoal.objId = "cube_BBCG";
      planGoal.action = "drop";
      planClient.sendGoal(planGoal);
      planClient.waitForResult();

      if(planClient.getResult()->error_code == 1)
      {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
      }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  }
  }
  }
  }
}

void scenario_rosbag(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("throw_box_left",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
    
     if (executeClient.getResult()->error_code == 1)
    {
      lookAt("box_B5",0,0,0);
      planGoal.planGroup = "right_arm";
      planGoal.objId = "cube_BGCB";
      planGoal.boxId = "box_B5";
      planGoal.action = "place";
      planClient.sendGoal(planGoal);
      planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

    if (executeClient.getResult()->error_code == 1)
    {
    planGoal.planGroup = "right_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "right_arm_home";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  
  }
  }
  }
  }
}

void scenario_multi_drop_arm(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  
  if (executeClient.getResult()->error_code == 1)
  {
    lookAt("throw_box_left",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_BGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  
   if (executeClient.getResult()->error_code == 1)
  {
    lookAt("throw_box_left",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BGCB";
    planGoal.action = "drop";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

    if (executeClient.getResult()->error_code== 1)
    {
      lookAt("cube_BBCG",0,0,0);
      planGoal.planGroup = "left_arm";
      planGoal.objId = "cube_BBCG";
      planGoal.action = "pick";
      planClient.sendGoal(planGoal);
      planClient.waitForResult();

      if(planClient.getResult()->error_code == 1)
      {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
      }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }

    if (executeClient.getResult()->error_code == 1)
    {
      lookAt("throw_box_left",0,0,0);
      planGoal.planGroup = "left_arm";
      planGoal.objId = "cube_BBCG";
      planGoal.action = "drop";
      planClient.sendGoal(planGoal);
      planClient.waitForResult();

      if(planClient.getResult()->error_code == 1)
      {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
      }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
      return;
    }
  }
  }
  }
  }
  }
}

void home_body(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time
  planGoal.planGroup = "whole_body";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "whole_body_home";

  planClient.sendGoal(planGoal);
  planClient.waitForResult();

  if(planClient.getResult()->error_code == 1)
  {
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionClient");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  char ch;


  moveit::planning_interface::MoveGroupInterface base("base");
  moveit::planning_interface::MoveGroupInterface::Plan p;

  base.setJointValueTarget ("base_x_joint",2.0);
  base.setJointValueTarget ("base_y_joint",-1);
  base.setJointValueTarget ("base_footprint_joint",0.0);
  base.plan(p);

  //actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/pr2_tasks_node/plan", true);

  //actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/pr2_tasks_node/execute", true);

  //home_body(plan,execute);

  //scenario_replace(plan,execute);
  



  ros::waitForShutdown();
  return 0;
}

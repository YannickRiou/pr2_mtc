#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

#include <geometry_msgs/PoseStamped.h>


#include <moveit_msgs/Grasp.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

// Dual arm related include
#include <move_group_x/move_group_x.h>

// Head related include
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <dt_head_gestures/HeadScanAction.h>

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
  ros::Duration(2).sleep(); 

}

void setGraspPose(moveit_msgs::Grasp& g, const std::string frame, const geometry_msgs::Pose p)
{
    g.id = "gl_" + frame;
    g.grasp_pose.header.frame_id = frame;
    g.grasp_pose.pose = p;
}

void waitUser(const std::string msg)
{
    do{
        // Use AINSI Escape code to make the text green in a Linux terminal
        std::cout<<"\033[1;32m\n"
                   "Press ENTER: "<<msg<<" \033[0m\n";
                   ros::spinOnce();
    }while (std::cin.get() != '\n');
}

void setPosePosition(geometry_msgs::Pose& p, const double x, const double y, const double z)
{
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
}

void setPoseRPY(geometry_msgs::Pose& p, const double roll, const double pitch, const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
}

void setGripperTranslation(moveit_msgs::GripperTranslation& gt, const std::string frame, const double des_d, const double min_d,
                           const double dir_x, const double dir_y, const double dir_z)
{
    gt.desired_distance = des_d;
    gt.min_distance = min_d;
    gt.direction.header.frame_id = frame;
    gt.direction.vector.x = dir_x;
    gt.direction.vector.y = dir_y;
    gt.direction.vector.z = dir_z;
}

void openGripper(trajectory_msgs::JointTrajectory& posture,std::string armUsed)
{
  if(armUsed == "left_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "l_gripper_joint";
    posture.joint_names[1] = "l_gripper_motor_screw_joint";
    posture.joint_names[2] = "l_gripper_l_finger_joint";
    posture.joint_names[3] = "l_gripper_r_finger_joint";
    posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.088;
    posture.points[0].positions[1] = 1;
    posture.points[0].positions[2] = 0.477;
    posture.points[0].positions[3] = 0.477;
    posture.points[0].positions[4] = 0.477;
    posture.points[0].positions[5] = 0.477;
    posture.points[0].time_from_start = ros::Duration(2);
  }
  else if(armUsed == "right_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "r_gripper_joint";
    posture.joint_names[1] = "r_gripper_motor_screw_joint";
    posture.joint_names[2] = "r_gripper_l_finger_joint";
    posture.joint_names[3] = "r_gripper_r_finger_joint";
    posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.088;
    posture.points[0].positions[1] = 1;
    posture.points[0].positions[2] = 0.477;
    posture.points[0].positions[3] = 0.477;
    posture.points[0].positions[4] = 0.477;
    posture.points[0].positions[5] = 0.477;
    posture.points[0].time_from_start = ros::Duration(2);
  }
}

void closedGripper(trajectory_msgs::JointTrajectory& posture,std::string armUsed)
{
  if(armUsed == "left_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "l_gripper_joint";
    posture.joint_names[1] = "l_gripper_motor_screw_joint";
    posture.joint_names[2] = "l_gripper_l_finger_joint";
    posture.joint_names[3] = "l_gripper_r_finger_joint";
    posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].positions[2] = 0.25;
    posture.points[0].positions[3] = 0.25;
    posture.points[0].positions[4] = 0.25;
    posture.points[0].positions[5] = 0.25;
    posture.points[0].time_from_start = ros::Duration(2);
  } else if(armUsed == "right_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "r_gripper_joint";
    posture.joint_names[1] = "r_gripper_motor_screw_joint";
    posture.joint_names[2] = "r_gripper_l_finger_joint";
    posture.joint_names[3] = "r_gripper_r_finger_joint";
    posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].positions[2] = 0.25;
    posture.points[0].positions[3] = 0.25;
    posture.points[0].positions[4] = 0.25;
    posture.points[0].positions[5] = 0.25;
    posture.points[0].time_from_start = ros::Duration(2);
  }

}

void setPoseOrientation(geometry_msgs::Pose& p, const double x, const double y, const double z, const double w)
{
    p.orientation.x = x;
    p.orientation.y = y;
    p.orientation.z = z;
    p.orientation.w = w;
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

   lookAt("cube_GGCB",0,0,0); 
 
  if (executeClient.getResult()->error_code== 1)
  {
    lookAt("cube_GGCB",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_GGCB";
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
    lookAt("box_C1",0,0,0);
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_BBCG";
    planGoal.boxId = "box_C1";
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
    lookAt("box_C2",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_GGCB";
    planGoal.boxId = "box_C2";
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

void pick_loop(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction>& planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time  

  for(int i =0; i=10;i++)
  {
    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_GBCG";
    planGoal.action = "pick";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }

    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    executeClient.sendGoal(executeGoal);
    executeClient.waitForResult();

    planGoal.planGroup = "left_arm";
    planGoal.objId = "cube_GBCG";
    planGoal.boxId = "box_C3";
    planGoal.action = "place";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }

    planGoal.planGroup = "left_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "left_arm_home";
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    executeClient.sendGoal(executeGoal);
    executeClient.waitForResult();
  }
}

void scan_room(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& navToClient, actionlib::SimpleActionClient<dt_head_gestures::HeadScanAction>& headScan)
{
  move_base_msgs::MoveBaseGoal navToGoal;

  dt_head_gestures::HeadScanGoal headGoal;

  for(int i=0;i<3;i++)
  {
    headGoal.central_point.header.frame_id = "base_footprint";
    headGoal.central_point.point.x = 0.7;
    headGoal.central_point.point.y = 0;
    headGoal.central_point.point.z = 0.80;
    headGoal.height = 0.6;
    headGoal.width = 0.6;
    headGoal.step_length = 0.3;
    headGoal.duration_per_point.data.sec = 2.3;
    headScan.sendGoal(headGoal);
    headScan.waitForResult();

    navToGoal.target_pose.header.frame_id = "base_footprint";
    navToGoal.target_pose.pose.orientation.x = 0.0;
    navToGoal.target_pose.pose.orientation.y = 0.0;
    navToGoal.target_pose.pose.orientation.z = 0.707;
    navToGoal.target_pose.pose.orientation.w = 0.707;

    navToClient.sendGoal(navToGoal);
    navToClient.waitForResult();

    ros::Duration(2).sleep();
  }
}


void scenario_dual_arm(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& navToClient, actionlib::SimpleActionClient<dt_head_gestures::HeadScanAction>& headScan)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;
  geometry_msgs::PoseStamped customPose;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Time timeZero(0.0);
  geometry_msgs::TransformStamped transform;

  move_base_msgs::MoveBaseGoal navToGoal;

  dt_head_gestures::HeadScanGoal headGoal;


  std::vector<moveit_msgs::RobotTrajectory> trajectories;
  dual_arm_msgs::DualArmPlaceLocation loc;
  moveit_msgs::MoveItErrorCodes e;
  moveit_dual_arm::MoveGroupX arms_torso("arms_torso");
  moveit::planning_interface::MoveGroupInterface l_arm("left_arm");

  ros::Duration(3).sleep();

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time
  navToClient.waitForServer();
  ROS_INFO("Server started successfully");

  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home_compact";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();


  // Scan des objets de toute la chambre
  scan_room(navToClient,headScan);


 // waitUser("To move in front of table 1");

  // Move to another room
  // Avant porte salon
  navToGoal.target_pose.header.frame_id = "map";
  navToGoal.target_pose.pose.position.x = 5.87582483292;
  navToGoal.target_pose.pose.position.y = 8.27849388123;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.z = 0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();


  // Passage de porte


  // Avant porte chambre
  navToGoal.target_pose.header.frame_id = "map";
  navToGoal.target_pose.pose.position.x = 5.87582483292;
  navToGoal.target_pose.pose.position.y = 9.98707675934;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.0;
  navToGoal.target_pose.pose.orientation.z = 0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();

  // Goal au centre de la chambre

  navToGoal.target_pose.header.frame_id = "map";
  navToGoal.target_pose.pose.position.x = 5.11309719086;
  navToGoal.target_pose.pose.position.y = 11.3950786591;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.0;
  navToGoal.target_pose.pose.orientation.z = 0.0;
  navToGoal.target_pose.pose.orientation.w = 1.0;

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();

  // Scan des objets de toute la chambre
  scan_room(navToClient,headScan);


  // Deplacement devant la table
  navToGoal.target_pose.header.frame_id = "table_2";
  navToGoal.target_pose.pose.position.x = 0.0;
  navToGoal.target_pose.pose.position.y = -0.90;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.00;
  navToGoal.target_pose.pose.orientation.z = 0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  // try{
  //  tfBuffer.canTransform("map", "table_2",timeZero, ros::Duration(5.0));
  //  transform = tfBuffer.lookupTransform("map", "table_2", timeZero);
  //  tf2::doTransform(navToGoal.target_pose, dockGoal.targetPose,transform);
  // }
  // catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();
  
  

  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();

  //waitUser("To pick cube");

  // scan de la table
  headGoal.central_point.header.frame_id = "base_footprint";
  headGoal.central_point.point.x = 0.7;
  headGoal.central_point.point.y = 0;
  headGoal.central_point.point.z = 0.45;
  headGoal.height = 0.2;
  headGoal.width = 0.5;
  headGoal.step_length = 0.1;
  headGoal.duration_per_point.data.sec = 1.8;
  headScan.sendGoal(headGoal);
  headScan.waitForResult();

  lookAt("cube_GBCG",0,0,0);

  planGoal.planGroup = "left_arm";
  planGoal.objId = "cube_GBCG";
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
    ROS_ERROR_STREAM("Retrying to " << planGoal.action << " " << planGoal.objId);
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
      return;
  }

  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();
 
  lookAt("cube_GBTG_2",0,0,0);
  planGoal.planGroup = "right_arm";
  planGoal.objId = "cube_GBTG_2";
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
    ROS_ERROR_STREAM("Retrying to " << planGoal.action << " " << planGoal.objId);
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
      return;
  }


  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home_compact";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();

  // Avant porte chambre
  navToGoal.target_pose.header.frame_id = "map";
  navToGoal.target_pose.pose.position.x = 5.87582483292;
  navToGoal.target_pose.pose.position.y = 9.98707675934;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.0;
  navToGoal.target_pose.pose.orientation.z = -0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();       

  // Passage de porte

  // Avant porte salon
  navToGoal.target_pose.header.frame_id = "map";
  navToGoal.target_pose.pose.position.x = 5.87582483292;
  navToGoal.target_pose.pose.position.y = 8.27849388123;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.0;
  navToGoal.target_pose.pose.orientation.z = -0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();

  // Scan de tout le salon
  scan_room(navToClient,headScan);

  // Deplacement devant la table basse
  navToGoal.target_pose.header.frame_id = "table_lack";
  navToGoal.target_pose.pose.position.x = 0.0;
  navToGoal.target_pose.pose.position.y = -0.70;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.00;
  navToGoal.target_pose.pose.orientation.z = 0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  //  try{
  //  tfBuffer.canTransform("map", "table_lack",timeZero, ros::Duration(5.0));
  //  transform = tfBuffer.lookupTransform("map", "table_lack", timeZero);
  //  tf2::doTransform(navToGoal.target_pose, dockGoal.targetPose,transform);
  // }
  // catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();

  
  
  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();
   

  headGoal.central_point.header.frame_id = "base_footprint";
  headGoal.central_point.point.x = 1.2;
  headGoal.central_point.point.y = 0;
  headGoal.central_point.point.z = 0.3;
  headGoal.height = 0.3;
  headGoal.width = 0.5;
  headGoal.step_length = 0.1;
  headGoal.duration_per_point.data.sec = 1.8;
  headScan.sendGoal(headGoal);
  headScan.waitForResult();

  lookAt("throw_box_pink",0,0,0);
  planGoal.planGroup = "left_arm";
  planGoal.objId = "cube_GBCG";
  planGoal.boxId = "throw_box_pink";
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
    ROS_ERROR_STREAM("Retrying to " << planGoal.action << " " << planGoal.objId);
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
      return;
  }

  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();

  lookAt("throw_box_pink",0,0,0);

  planGoal.action = "updateWorld";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();

  planGoal.planGroup = "right_arm";
  planGoal.objId = "cube_GBTG_2";
  planGoal.boxId = "throw_box_pink";
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
    ROS_ERROR_STREAM("Retrying to " << planGoal.action << " " << planGoal.objId);
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
      return;
  }

  planGoal.planGroup = "arms";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "arms_home";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();


  //waitUser("To pick the box with the both arm");

  // dual arm pick 
  lookAt("throw_box_pink",0,0,0);
  planGoal.planGroup = "left_arm+right_arm";
  planGoal.objId = "throw_box_pink";
  planGoal.action = "pickDual";

  customPose.header.frame_id = "throw_box_pink";
  customPose.pose.position.x = -0.185;
  customPose.pose.position.y = -0.05;
  customPose.pose.position.z = 0.20;
  customPose.pose.orientation.x = -0.500;
  customPose.pose.orientation.y = 0.500;
  customPose.pose.orientation.z = 0.500;
  customPose.pose.orientation.w = 0.500;

  planGoal.pose = customPose;
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
    ROS_ERROR_STREAM("Retrying to " << planGoal.action << " " << planGoal.objId);
    planClient.sendGoal(planGoal);
    planClient.waitForResult();
    if(planClient.getResult()->error_code == 1)  
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
      return;
  
  // Specifiy the desired value of the torso for the goal state.
  loc.user_goal_constr.joint_constraints.resize(2);

  moveit_msgs::JointConstraint& jc_torso = loc.user_goal_constr.joint_constraints[0];
  jc_torso.joint_name = "torso_lift_joint";
  jc_torso.position = 0.15;
  jc_torso.tolerance_below = 0.1;
  jc_torso.tolerance_above = 0.1;
  jc_torso.weight = 1.;

  //waitUser("To verify if cubes are in the box");


  // dual Arm place 
  arms_torso.setPlannerId("RRTConnectkConfigDualArm");

  loc.id = "loc_da_1";
  loc.first_eef_link = "l_wrist_roll_link";
  loc.second_eef_link = "r_wrist_roll_link";

  //openGripper(loc.first_post_place_posture,"left_arm");
  //setGripperTranslation(loc.first_post_place_retreat, "throw_box_pink", 0.15, 0.1, 0., 0., 1.);

  //openGripper(loc.second_post_place_posture,"right_arm");
  //setGripperTranslation(loc.second_post_place_retreat, "throw_box_pink", 0.15, 0.1, 0., 0., 1.);


  // Set the dual arm place location. This defines the desired pose of the frame "frame_to_place" in the world_frame.
  // So at the end of the dual arm place the "frame_to_place" will be at loc.place_pose.
  loc.place_pose.header.frame_id = "throw_box_pink";
  setPosePosition(loc.place_pose.pose, 0.0, 0.0, 0.40);
  setPoseRPY(loc.place_pose.pose, 0, 0., 0.);
  loc.frame_to_place = "throw_box_pink";

  // Set the start translation of the left gripper (as frame_to_place is the left gripper tool frame).
  // In some case the part can be in a support (slot) and this starting translation allows to extract the part from this support.
  setGripperTranslation(loc.start_place_translation, "base_footprint", 0.1, 0.05, 0., 0., 1.);

  // Set the approach of the left_gripper (as frame_to_place is the left gripper tool frame).
  setGripperTranslation(loc.pre_place_approach, "base_footprint", 0.1, 0.05, 0., 0., 1.);

  // Send the request to the dual arm place action server
  arms_torso.dual_arm_place("throw_box_pink", std::vector<dual_arm_msgs::DualArmPlaceLocation>(1, loc), trajectories);

  navToGoal.target_pose.header.frame_id = "table_1";
  navToGoal.target_pose.pose.position.x = 0.0;
  navToGoal.target_pose.pose.position.y = -0.70;
  navToGoal.target_pose.pose.position.z = 0.0;
  navToGoal.target_pose.pose.orientation.x = 0.0;
  navToGoal.target_pose.pose.orientation.y = 0.00;
  navToGoal.target_pose.pose.orientation.z = 0.707;
  navToGoal.target_pose.pose.orientation.w = 0.707;

  // try{
  // tfBuffer.canTransform ("map", "table_1",timeZero, ros::Duration(5.0));
  // transform = tfBuffer.lookupTransform("map", "table_1", timeZero);
  // tf2::doTransform(navToGoal.target_pose, dockGoal.targetPose,transform);
  // }
  // catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }

  navToClient.sendGoal(navToGoal);
  navToClient.waitForResult();

  //waitUser("To place the box with the both arm");

  // dual Arm place 
  arms_torso.setPlannerId("RRTConnectkConfigDualArm");

  loc.id = "loc_da_1";
  loc.first_eef_link = "l_wrist_roll_link";
  loc.second_eef_link = "r_wrist_roll_link";

  openGripper(loc.first_post_place_posture,"left_arm");
  setGripperTranslation(loc.first_post_place_retreat, "throw_box_pink", 0.15, 0.1, 0., 0., 1.);

  openGripper(loc.second_post_place_posture,"right_arm");
  setGripperTranslation(loc.second_post_place_retreat, "throw_box_pink", 0.15, 0.1, 0., 0., 1.);


  // Set the dual arm place location. This defines the desired pose of the frame "frame_to_place" in the world_frame.
  // So at the end of the dual arm place the "frame_to_place" will be at loc.place_pose.
  loc.place_pose.header.frame_id = "table_1";
  setPosePosition(loc.place_pose.pose, 0.0, 0.0, 0.10);
  setPoseRPY(loc.place_pose.pose, 0, 0., 0.);
  loc.frame_to_place = "throw_box_pink";

  jc_torso.joint_name = "torso_lift_joint";
  jc_torso.position = 0.33;
  jc_torso.tolerance_below = 0.1;
  jc_torso.tolerance_above = 0.1;
  jc_torso.weight = 1.;

  // moveit_msgs::JointConstraint& jc_gripper = loc.user_goal_constr.joint_constraints[1];
  // jc_gripper.joint_name = "l_wrist_flex_joint";
  // jc_gripper.position = -1.27;
  // jc_gripper.tolerance_below = 0.1;
  // jc_gripper.tolerance_above = 0.1;
  // jc_gripper.weight = 1.;

  // Set the start translation of the left gripper (as frame_to_place is the left gripper tool frame).
  // In some case the part can be in a support (slot) and this starting translation allows to extract the part from this support.
  setGripperTranslation(loc.start_place_translation, "base_footprint", 0.1, 0.05, 0., 0., 1.);

  // Set the approach of the left_gripper (as frame_to_place is the left gripper tool frame).
  setGripperTranslation(loc.pre_place_approach, "base_footprint", 0.1, 0.05, 1., 0., -1.);

  // Send the request to the dual arm place action server
  arms_torso.dual_arm_place("throw_box_pink", std::vector<dual_arm_msgs::DualArmPlaceLocation>(1, loc), trajectories);
  
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
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle n;

  char ch;

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/pr2_tasks_node/plan", true);

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/pr2_tasks_node/execute", true);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navTo("/move_base", true);
  navTo.waitForServer();

  actionlib::SimpleActionClient<dt_head_gestures::HeadScanAction> headScan("/head_scan/head_scan", true);
  headScan.waitForServer();

 
  geometry_msgs::PoseStamped goalPose;
  //ros::Subscriber moveBasePoseGoalSub = n.subscribe<geometry_msgs::PoseStamped>("/move_base/current_goal", 1,boost::bind(&moveBasePoseCallback,_1,&dockTo, &navTo));
  //ros::Duration(2).sleep(); 
  //home_body(plan,execute);


  scenario_dual_arm(plan,execute,navTo,headScan);

  //scan_room(navTo,headScan);

  //pick_loop(plan,execute);
  
  //scenario_replace_2(plan,execute);

  return 0;
}

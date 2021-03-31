#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>


// Dual arm related include
#include <move_group_x/move_group_x.h>

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


void scenario_dual_arm(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;
  geometry_msgs::PoseStamped customPose;

  std::vector<moveit_msgs::RobotTrajectory> trajectories;
  moveit_dual_arm::MoveGroupX arms_torso("arms_torso");


  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  {
    lookAt("cube_BBTG",0,0,0);
    planGoal.planGroup = "right_arm";
    planGoal.objId = "cube_BBTG";
    planGoal.action = "pickAuto";
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
    lookAt("throw_box_pink",0,0,0);
    planGoal.planGroup = "right_arm";
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

    if (executeClient.getResult()->error_code == 1)
    {
      lookAt("cube_GBTB",0,0,0);
      planGoal.planGroup = "right_arm";
      planGoal.objId = "cube_GBTB";
      planGoal.action = "pickAuto";
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
        lookAt("throw_box_pink",0,0,0);
        planGoal.planGroup = "right_arm";
        planGoal.objId = "cube_GBTB";
        planGoal.action = "drop";
        planClient.sendGoal(planGoal);
        planClient.waitForResult();


        // dual arm pick 
        lookAt("throw_box_pink",0,0,0);
        planGoal.planGroup = "left_arm+right_arm";
        planGoal.objId = "throw_box_pink";
        planGoal.action = "pickDual";

        customPose.header.frame_id = "throw_box_pink";
        customPose.pose.position.x = -0.185;
        customPose.pose.position.y = -0.10;
        customPose.pose.position.z = 0.230;
        customPose.pose.orientation.x = -0.500;
        customPose.pose.orientation.y = 0.500;
        customPose.pose.orientation.z = 0.500;
        customPose.pose.orientation.w = 0.500;

        planGoal.pose = customPose;
        planClient.sendGoal(planGoal);
        planClient.waitForResult();

        if (executeClient.getResult()->error_code == 1)
        {

          // TODO: Deplacement devant pos_2 marker

          // dual Arm place 
          arms_torso.setPlannerId("RRTConnectkConfigDualArm");

          dual_arm_msgs::DualArmPlaceLocation loc;
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
          setPosePosition(loc.place_pose.pose, 0.0, 0.0, 0.05);
          setPoseRPY(loc.place_pose.pose, 0, 0., 0.);
          loc.frame_to_place = "throw_box_pink";

          // Set the start translation of the left gripper (as frame_to_place is the left gripper tool frame).
          // In some case the part can be in a support (slot) and this starting translation allows to extract the part from this support.
          setGripperTranslation(loc.start_place_translation, "base_footprint", 0.1, 0.05, 0., 0., 1.);

          // Set the approach of the left_gripper (as frame_to_place is the left gripper tool frame).
          setGripperTranslation(loc.pre_place_approach, "base_footprint", 0.1, 0.05, 1., 0., 0.);

          // Send the request to the dual arm place action server
          arms_torso.dual_arm_place("throw_box_pink", std::vector<dual_arm_msgs::DualArmPlaceLocation>(1, loc), trajectories);


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

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/pr2_tasks_node/plan", true);

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/pr2_tasks_node/execute", true);

  home_body(plan,execute);

  scenario_dual_arm(plan,execute);
  



  ros::waitForShutdown();
  return 0;
}

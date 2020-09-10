#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_mtc/getPose.h>



bool getPose(pr2_mtc::getPose::Request  &req, pr2_mtc::getPose::Response &res)
{
    geometry_msgs::PoseStamped pose_boxC5;
    pose_boxC5.header.frame_id = "base_footprint";
    pose_boxC5.pose.position.x = 0.925;
    pose_boxC5.pose.position.y = 0.057;
    pose_boxC5.pose.position.z = 0.82;      
    pose_boxC5.pose.orientation.x = 0.0;  
    pose_boxC5.pose.orientation.y = 0.0;  
    pose_boxC5.pose.orientation.z = -0.707; 
    pose_boxC5.pose.orientation.w = 0.707;  

	geometry_msgs::PoseStamped pose_boxB5;
	pose_boxB5.header.frame_id = "base_footprint";
	pose_boxB5.pose.position.x = 0.925;
	pose_boxB5.pose.position.y = 0.052;
	pose_boxB5.pose.position.z = 1.062;      
	pose_boxB5.pose.orientation.x = 0.0;  
	pose_boxB5.pose.orientation.y = 0.0;  
	pose_boxB5.pose.orientation.z = 0.707; 
	pose_boxB5.pose.orientation.w = 0.707;  

	geometry_msgs::PoseStamped table_1;
	table_1.header.frame_id = "base_footprint";
	table_1.pose.position.x = 1.02;
	table_1.pose.position.y = 0.0;
	table_1.pose.position.z = 0.7188;      
	table_1.pose.orientation.x = 0.0;  
	table_1.pose.orientation.y = 0.0;  
	table_1.pose.orientation.z = 0.0; 
	table_1.pose.orientation.w = 1.0;  

	geometry_msgs::PoseStamped pose_cube_GBTG_2;
	pose_cube_GBTG_2.header.frame_id = "base_footprint";
	pose_cube_GBTG_2.pose.position.x = 0.87;
	pose_cube_GBTG_2.pose.position.y = 0.063;
	pose_cube_GBTG_2.pose.position.z = 1.04;      
	pose_cube_GBTG_2.pose.orientation.x = 0.0;  
	pose_cube_GBTG_2.pose.orientation.y = 0.0;  
	pose_cube_GBTG_2.pose.orientation.z = 0.0; 
	pose_cube_GBTG_2.pose.orientation.w = 1.0;  

	geometry_msgs::PoseStamped pose_cube_GBCB;
	pose_cube_GBCB.header.frame_id = "";
	pose_cube_GBCB.pose.position.x = 0.87;
	pose_cube_GBCB.pose.position.y = 0.069;
	pose_cube_GBCB.pose.position.z = 0.79;      
	pose_cube_GBCB.pose.orientation.x = 0.0;  
	pose_cube_GBCB.pose.orientation.y = 0.0;  
	pose_cube_GBCB.pose.orientation.z = 0.0; 
	pose_cube_GBCB.pose.orientation.w = 1.0;  

    for(int i=0; i<req.ids.size(); i++)
    {
        if(req.ids[i] == "box_B5")
        {
            res.poses.push_back(pose_boxB5);
        }
        else if(req.ids[i] == "box_C5")
        {
            res.poses.push_back(pose_boxC5);
        }
        else if(req.ids[i] == "cube_GBTG_2")
        {
            res.poses.push_back(pose_cube_GBTG_2);
        }
        else if(req.ids[i] == "cube_GBCB")
        {
            res.poses.push_back(pose_cube_GBCB);
        }
        else if(req.ids[i] == "table_1")
        {
            res.poses.push_back(table_1);
        }
    }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underworldServer");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  ros::ServiceServer service = nh.advertiseService("getPose", getPose);
  
  while(ros::ok());

  return 0;
}

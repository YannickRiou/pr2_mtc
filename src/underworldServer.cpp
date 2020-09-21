#include "ros/ros.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_motion_tasks_msgs/GetPose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped pose_boxC5;
geometry_msgs::PoseStamped pose_boxB5;
geometry_msgs::PoseStamped pose_boxC1;
geometry_msgs::PoseStamped table_1;
geometry_msgs::PoseStamped pose_cube_GBTG_2;
geometry_msgs::PoseStamped pose_cube_GBCB;

bool GetPose(pr2_motion_tasks_msgs::GetPose::Request  &req, pr2_motion_tasks_msgs::GetPose::Response &res)
{
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
        else if(req.ids[i] == "box_C1")
        {
            res.poses.push_back(pose_boxC1);
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

void markerCallback(const visualization_msgs::MarkerConstPtr& marker, const geometry_msgs::TransformStamped transform)
{

    geometry_msgs::PoseStamped colliObjPosetransformed;
    geometry_msgs::PoseStamped colliObjPoseUntransformed;

     // Define a pose for the object (specified relative to frame_id)


    // From ar_track_alvar
    if(marker->id == 54)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        table_1.pose = colliObjPosetransformed.pose;

        table_1.header.frame_id = "base_footprint";
        table_1.pose.orientation.x = 0.0;
        table_1.pose.orientation.y = 0.0;
        table_1.pose.orientation.z = 0.0;
        table_1.pose.orientation.w = 1.0;
    }
    else if(marker->id == 100)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        pose_boxC5.pose = colliObjPosetransformed.pose;

        pose_boxC5.header.frame_id = "base_footprint";
        pose_boxC5.pose.orientation.x = 0.0;
        pose_boxC5.pose.orientation.y = 0.0;
        pose_boxC5.pose.orientation.z = -0.707;
        pose_boxC5.pose.orientation.w = 0.707;
    }
    else if(marker->id == 101)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        pose_boxB5.pose = colliObjPosetransformed.pose;

        pose_boxB5.header.frame_id = "base_footprint";
        pose_boxB5.pose.orientation.x = 0.0;
        pose_boxB5.pose.orientation.y = 0.0;
        pose_boxB5.pose.orientation.z = -0.707;
        pose_boxB5.pose.orientation.w = 0.707;
    }
    else if(marker->id == 102)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        pose_boxC1.pose = colliObjPosetransformed.pose;

        pose_boxC1.header.frame_id = "base_footprint";
        pose_boxC1.pose.orientation.x = 0.0;
        pose_boxC1.pose.orientation.y = 0.0;
        pose_boxC1.pose.orientation.z = -0.707;
        pose_boxC1.pose.orientation.w = 0.707;
    }
    else if(marker->id == 230)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        pose_cube_GBTG_2.pose = colliObjPosetransformed.pose;

        pose_cube_GBTG_2.header.frame_id = "base_footprint";
        pose_cube_GBTG_2.pose.position.z = pose_cube_GBTG_2.pose.position.z - (0.055/2)+0.01;
        pose_cube_GBTG_2.pose.position.x = pose_cube_GBTG_2.pose.position.x + 0.055;
        pose_cube_GBTG_2.pose.orientation.x = 0.0;
        pose_cube_GBTG_2.pose.orientation.y = 0.0;
        pose_cube_GBTG_2.pose.orientation.z = 0.0;
        pose_cube_GBTG_2.pose.orientation.w = 1.0;
    }
    else if(marker->id == 206)
    {
        colliObjPoseUntransformed.pose = marker->pose;
        tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);
        pose_cube_GBCB.pose = colliObjPosetransformed.pose;

        pose_cube_GBCB.header.frame_id = "base_footprint";
        pose_cube_GBCB.pose.position.z = pose_cube_GBCB.pose.position.z - (0.055/2)+0.01;
        pose_cube_GBCB.pose.position.x = pose_cube_GBCB.pose.position.x + 0.055;
        pose_cube_GBCB.pose.orientation.x = 0.0;
        pose_cube_GBCB.pose.orientation.y = 0.0;
        pose_cube_GBCB.pose.orientation.z = 0.0;
        pose_cube_GBCB.pose.orientation.w = 1.0;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underworldServer");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::TransformStamped transform;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tableListenner(tfBuffer);

  ros::Duration(5).sleep();

  try
  {
    transform = tfBuffer.lookupTransform("base_footprint","head_mount_kinect2_rgb_optical_frame",ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(markerCallback,_1,transform));

  ros::ServiceServer service = nh.advertiseService("GetPose", GetPose);

  while(ros::ok());

  return 0;
}

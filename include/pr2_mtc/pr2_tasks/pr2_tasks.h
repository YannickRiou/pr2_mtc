#pragma once

#include <ros/ros.h>

#include <boost/thread/thread.hpp>

// Moveit task constructor related include
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_custom_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/cost_terms.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


// Moveit  related include
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/CollisionObject.h>


// TF2 related include
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Geometry messages related include
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

// standard Messages related include
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <shape_msgs/SolidPrimitive.h>
// Standard type related include
#include<string>
#include <unordered_map>

// Action related include
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

// Geometric shapes related include
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>

#include "ontologenius/OntologyManipulator.h"

// Service to get object pose from UDWS
#include <pr2_motion_tasks_msgs/GetPose.h>

// Action server for supervisor to call pick, place and move tasks
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Used to ask ontology to get all object on this support surface
#define ROBOT_ONTOLOGY_NAME "pr2_robot"

#define PLANNER "RRTConnect"

#define DEFAULT_LONGEST_VALID_SEGMENT_FRACTION 0.00001

#define GET_POSE_TOPIC "/overworld/getPose"

using namespace moveit::task_constructor;

class motionPlanning
{
    public:
        motionPlanning(ros::NodeHandle& nh);
        ~motionPlanning();

        void createPlaceTask(std::unique_ptr<moveit::task_constructor::Task>& placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses);

        void createMoveTask(std::unique_ptr<moveit::task_constructor::Task>& moveTask, const std::string planGroup, const geometry_msgs::PoseStamped moveToPose);

        void createMovePredefinedTask(std::unique_ptr<moveit::task_constructor::Task>& moveTask, const std::string planGroup,const std::string pose_id);

        void createPickTaskCustom(std::unique_ptr<moveit::task_constructor::Task>& pickTask, const std::string planGroup,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses);

        void createPickTaskCustomDual(std::unique_ptr<moveit::task_constructor::Task>& pickTask, const std::string planGroup_first,const std::string planGroup_second ,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses_first, std::vector<geometry_msgs::PoseStamped> graspPoses_second);

        void createPickTask(std::unique_ptr<moveit::task_constructor::Task>& pickTask, const std::string planGroup,const std::string object, const std::string supportId);

        void createDropTask(std::unique_ptr<moveit::task_constructor::Task>& dropTask, const std::string planGroup,const std::string object, const std::string boxId);

        int updateWorld(ros::ServiceClient& udwClient);

        void getPoseIntoBasefootprint(const geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped& out_pose);

        void planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& udwClient);

        void executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal);

        void taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat);




    private:
        ros::NodeHandle nh_;

        // Variable to store the last task that was planned
        // To be able to execute it afterward
        std::unique_ptr<Task> lastPlannedTask_;
        std::string taskArmGroup_;
        std::string taskObjId_;


        std::unique_ptr<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>> planServer_;
        std::unique_ptr<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>> executeServer_;

        // Service to get object pose from underworld
        ros::ServiceClient getPoseSrv_;

        OntologyManipulator onto_;

        geometry_msgs::TransformStamped mainTransform_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transformListenner_;

        // List to keep track of mesh/obj association to avoid asking for mesh
        // if we already have it
        std::unordered_map<std::string, std::string> objMeshMap_;

        // Define PlanningSceneInterface object to add and remove collision objects
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        // Robot model shared by all tasks
        robot_model_loader::RobotModelLoader robot_model_loader_;
        moveit::core::RobotModelPtr kinematic_model_;

        // planner used for approach and retreat
        std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesianPlanner_;

        // planner used for connect
        std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> pipelinePlanner_;

        // planner used for gripper open/close movements
        std::shared_ptr<solvers::JointInterpolationPlanner> gripper_planner_;

        // Variable to set the eef used during task
        std::string eef_;
        std::string first_eef_;
        std::string second_eef_;

        // Variable to set the ikFrame used during task
	    std::string ikFrame_;
        std::string first_ikFrame_;
        std::string second_ikFrame_;

};

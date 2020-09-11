#pragma once

#include <ros/ros.h>

#include <boost/thread/thread.hpp>

// Moveit task constructor related include
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Moveit  related include
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
#include <pr2_mtc/getPose.h>

// Action server for supervisor to call pick, place and move tasks
#include <pr2_mtc/planAction.h>
#include <pr2_mtc/executeAction.h>

// Used to ask ontology to get all object on this support surface
#define SUPPORT_SURFACE "table_1"

using namespace moveit::task_constructor;

class motionPlanning
{
    public:
        explicit motionPlanning();
        ~motionPlanning();

        void createPlaceTask(Task &placeTask, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose);

        void createMoveTask(Task &moveTask, const std::string planGroup, const geometry_msgs::PoseStamped moveToPose);

        void createPickTaskCustom(Task &pickTask, const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose);

        void createPickTask(Task &pickTask, const std::string planGroup,const std::string object);

        void updateWorld(ros::ServiceClient& udwClient);

        void planCallback(const pr2_mtc::planGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::planAction>* planServer, ros::ServiceClient& udwClient);

        void executeCallback(const pr2_mtc::executeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::executeAction>* executeServer);



    private:
        ros::NodeHandle nh_;

        // Variable to store the last task that was planned 
        // To be able to execute it afterward
        std::shared_ptr<Task> lastPlannedTask_;

        OntologyManipulator onto_;
        
        geometry_msgs::TransformStamped mainTransform_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transformListenner_;

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

        std::shared_ptr<solvers::JointInterpolationPlanner> gripper_planner_;

        




};


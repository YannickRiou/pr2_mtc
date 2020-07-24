#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include<string>

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pr2_mtc/pr2perception.h>

using namespace moveit::task_constructor;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

void createPlaceTask(Task &placeTask, const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian, const moveit::core::RobotModelPtr& robotModel, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose);

void createMoveTask(Task &moveTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup, const geometry_msgs::PoseStamped moveToPose);

void createPickTaskCustom(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose);

void createPickTask(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel, const std::string planGroup,const std::string object);

bool gripper_open(GripperClient* gripper);

bool gripper_close(GripperClient* gripper, float effort);

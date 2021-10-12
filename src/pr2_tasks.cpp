/**
 * \file pr2_tasks.cpp
 * \brief Handling of PR2 tasks creation with helps of Moveit Task Constructor
 * \author Yannick R.
 * \version 0.1
 * \date 22/08/20
 *
 * Allow to create multiple tasks
 *
 */

#include <pr2_mtc/pr2_tasks/pr2_tasks.h>

// Class constructor
motionPlanning::motionPlanning(ros::NodeHandle& nh)
  : nh_(nh),
  	onto_(ROBOT_ONTOLOGY_NAME),
	robot_model_loader_("robot_description"),
	transformListenner_(tfBuffer_)
{
	ROS_INFO("[Entering class constructor]");

	// Close ontology
	onto_.close();

	// Set to verbose when debugging
	//onto_.verbose(true);

	// Load the common kinematic model of the robot that will be used when creating a task with MTC
	kinematic_model_ = robot_model_loader_.getModel();

	//auto g_planning_scene = std::make_unique<planning_scene::PlanningScene>(robot_model_loader_.getModel());

	//g_planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true /* exclusive */);

	// Create the common cartesian planner that will be used for translation movement when creating task with MTC
	cartesianPlanner_ = std::make_shared<solvers::CartesianPath>();
	cartesianPlanner_->setProperty("jump_threshold", 0.00);

	// Create the common pipeline planner (by default RRTConnect) that will be used when creating a task with MTC
	pipelinePlanner_ = std::make_shared<solvers::PipelinePlanner>();
	pipelinePlanner_->setPlannerId(PLANNER);

	// Define the distance between two waypoint in trajectory.
	// Large value might lead to a trajectory going through collision objects
	// Small value will increase computing time
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",DEFAULT_LONGEST_VALID_SEGMENT_FRACTION);

	// Create the common planner for gripper open/close movement when creating task with MTC
	gripper_planner_ = std::make_shared<solvers::JointInterpolationPlanner>();

	getPoseSrv_ = nh_.serviceClient<pr2_motion_tasks_msgs::GetPose>(GET_POSE_TOPIC);
	ros::service::waitForService(GET_POSE_TOPIC, -1);

	ROS_INFO("[Node connected to getPose service]");

	getBoundingBoxSrv_ = nh_.serviceClient<overworld::BoundingBox>(GET_BOUNDINGBOX_TOPIC);
	ros::service::waitForService(GET_BOUNDINGBOX_TOPIC, -1);

	ROS_INFO("[Node connected to getBoundingBox service]");

	planServer_ = std::make_unique<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>>(nh_, "plan", boost::bind(&motionPlanning::planCallback,this, _1, getPoseSrv_), false);
	executeServer_ =  std::make_unique<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>>(nh_, "execute", boost::bind(&motionPlanning::executeCallback,this, _1), false);


	// Action servers for supervisor
	planServer_->start();
	executeServer_->start();
	
	ROS_INFO("[Plan and execute action servers started successfully]");


	debug_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pr2_debug", 10, true);
}

// Class destructor
motionPlanning::~motionPlanning()
{
}

 /**
 * \fn void createPlaceTask(Task &placeTask, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose)
 * \brief Function to create a place task with specific pose and object
 *
 * \param placeTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to be placed (it needs to be already attached to the eef of the associated planGroup)
 * \param placePose Pose where to place the object
 */
void motionPlanning::createPlaceTask(std::unique_ptr<moveit::task_constructor::Task>& placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses)
{
	placeTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string ungrasp;

	placeTask->setProperty("object",object);

	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "gripper_upright";
	upright_constraint.orientation_constraints.resize(1);
	moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
	c.header.frame_id= "base_footprint";
	c.orientation.x= 0.0;
	c.orientation.y= 0.0;
	c.orientation.z= 0.0;
	c.orientation.w= 1.0;
	c.absolute_x_axis_tolerance= 0.3925;
	c.absolute_y_axis_tolerance= 0.3925;
	c.absolute_z_axis_tolerance= 0.785;
	c.weight= 1.0;

	if(planGroup == "left_arm")
	{
		placeTask->setProperty("group",planGroup);
		placeTask->setProperty("eef","left_gripper");
	    eef_ = "left_gripper";
		ikFrame_ = "l_gripper_tool_frame";
		ungrasp = "left_open";

		c.link_name= "l_gripper_tool_frame";

	}
	else if(planGroup == "right_arm")
	{
		placeTask->setProperty("group",planGroup);
		placeTask->setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		ikFrame_ = "r_gripper_tool_frame";
		ungrasp = "right_open";

		c.link_name= "r_gripper_tool_frame";

	}

	// Increase precision for place to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);

	placeTask->setProperty("ik_frame",ikFrame_);

	pipelinePlanner_->setPlannerId(PLANNER);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	// Copy properties defined for placeTask to initial stages (then it will be possible to get them into other stages)
	placeTask->properties().exposeTo(initial->properties(), { "eef", "group","ik_frame" });
	placeTask->add(std::move(initial));

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect to place", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		//connect->setPathConstraints(upright_constraint);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		placeTask->add(std::move(connect));
	}

	{
		auto place = std::make_unique<SerialContainer>("place object");

		placeTask->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
		place->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});

		{
			auto stage = std::make_unique<stages::GenerateCustomPose>("place the object");
			stage->setCustomPoses(placePoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame_);
			// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(ungrasp);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object, ikFrame_);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat from object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.02, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}
		placeTask->add(std::move(place));
	}
}

 /**
 * \fn void createMoveTask(Task &moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
 * \brief Function to create a move task with specific pose and planGroup (arm)
 *
 * \param moveTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param moveToPose Pose where to place the frame (r_gripper_tool_frame or l_gripper_tool_frame depending on planGroup)
 */
void motionPlanning::createMoveTask(std::unique_ptr<moveit::task_constructor::Task>&moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
{
	moveTask->setRobotModel(kinematic_model_);

	if(planGroup == "left_arm")
	{
		ikFrame_ = "l_gripper_tool_frame";
		eef_ = "left_gripper";
	}
	else if(planGroup == "right_arm")
	{
		ikFrame_ = "r_gripper_tool_frame";
		eef_ = "right_gripper";
	}

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	moveTask->add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		moveTask->add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef_);
		stage->setPose(moveToPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(10);
		wrapper->setIKFrame(ikFrame_);
		wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef_);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		moveTask->add(std::move(wrapper));
	}
}

 /**
 * \fn void createMovePredefinedTask(Task &moveTask, const std::string planGroup,const std::string pose_id)
 * \brief Function to create a move task with specific pre-defined pose and planGroup (arm)
 *
 * \param moveTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param pose_id Id of the pose
 */
void motionPlanning::createMovePredefinedTask(std::unique_ptr<moveit::task_constructor::Task>&moveTask, const std::string planGroup,const std::string pose_id)
{
	moveTask->setRobotModel(kinematic_model_);

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);
	pipelinePlanner_->setPlannerId(PLANNER);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	moveTask->add(std::move(initial));

	{
		auto stage = std::make_unique<stages::MoveTo>("Move to pre-defined pose", pipelinePlanner_);
		stage->setGroup(planGroup);
		stage->setGoal(pose_id);
		moveTask->add(std::move(stage));
	}

}

 /**
 * \fn void createDropTask(Task &dropTask, const std::string planGroup,const std::string object)
 * \brief Function to create a drop task
 *
 * \param dropTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to be droped (it needs to be already attached to the eef of the associated planGroup)
 */
void motionPlanning::createDropTask(std::unique_ptr<moveit::task_constructor::Task>& dropTask, const std::string planGroup,const std::string object, const std::string dropFrame)
{
	dropTask->setRobotModel(kinematic_model_);

	std::string ungrasp;

	geometry_msgs::PoseStamped dropPose;



	if(planGroup == "left_arm")
	{
		ikFrame_ = "l_gripper_tool_frame";
		eef_ = "left_gripper";
		ungrasp = "left_open";
	}
	else if(planGroup == "right_arm")
	{
		ikFrame_ = "r_gripper_tool_frame";
		eef_ = "right_gripper";
		ungrasp = "right_open";
	}

	std::vector<geometry_msgs::PoseStamped> dropPoses;
	geometry_msgs::PoseStamped customDropPose;

	for(float i=0;i<0.6;i=i+0.05)
	{
		customDropPose.header.frame_id = dropFrame;
		customDropPose.pose.position.x = 0.0;
		customDropPose.pose.position.y = 0.0;
		customDropPose.pose.position.z = i;
		customDropPose.pose.orientation.x = 0.0;
		customDropPose.pose.orientation.y = 0.707;
		customDropPose.pose.orientation.z = 0.0;
		customDropPose.pose.orientation.w = 0.707;
		dropPoses.push_back(customDropPose);

		customDropPose.header.frame_id = dropFrame;
		customDropPose.pose.position.x = 0.0;
		customDropPose.pose.position.y = 0.0;
		customDropPose.pose.position.z = i;
		customDropPose.pose.orientation.x = 0.0;
		customDropPose.pose.orientation.y = 0.0;
		customDropPose.pose.orientation.z = 0.707;
		customDropPose.pose.orientation.w = 0.707;
		dropPoses.push_back(customDropPose);
	}	



	// Increase precision for drop to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);

	pipelinePlanner_->setPlannerId(PLANNER);


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	dropTask->add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->properties().configureInitFrom(Stage::PARENT);
		dropTask->add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GenerateCustomPose>("Generate Custom Poses");
		stage->setCustomPoses(dropPoses);
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef_);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(10);
		wrapper->setIKFrame(ikFrame_);
		wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef_);
		dropTask->add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
		stage->setGroup(eef_);
		stage->setGoal(ungrasp);
		dropTask->add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, ikFrame_);
		dropTask->add(std::move(stage));
	}

}

 /**
 * \fn void createPickTaskCustom(Task &pickTask, const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose)
 * \brief Function to create a pick task with specific grasp pose and planGroup (arm)
 *
 * \param pickTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 * \param graspPose Grasp pose to be used when picking the object
 */
void motionPlanning::createPickTaskCustom(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses)
{
	pickTask->setRobotModel(kinematic_model_);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// Property and variable definitions
	std::string pregrasp;
	std::string postgrasp;

	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "gripper_upright";
	upright_constraint.orientation_constraints.resize(1);
	moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
	c.header.frame_id= "base_footprint";
	c.orientation.x= 0.0;
	c.orientation.y= 0.0;
	c.orientation.z= 0.0;
	c.orientation.w= 1.0;
	c.absolute_x_axis_tolerance= 0.3925;
	c.absolute_y_axis_tolerance= 0.3925;
	c.absolute_z_axis_tolerance= 0.785;
	c.weight= 1.0;

	pickTask->setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask->setProperty("group",planGroup);
		pickTask->setProperty("eef","left_gripper");
	    eef_ = "left_gripper";
		pregrasp = "left_open";
		postgrasp = "left_close";
		ikFrame_ = "l_gripper_tool_frame";

		c.link_name= "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask->setProperty("group",planGroup);
		pickTask->setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		pregrasp = "right_open";
		postgrasp = "right_close";
		ikFrame_ = "r_gripper_tool_frame";

		c.link_name= "r_gripper_tool_frame";
	}

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(PLANNER);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask->add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", gripper_planner_);
		stage->setGroup(eef_);
		stage->setGoal(pregrasp);
		current_state = stage.get();
		pickTask->add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		//connect->setPathConstraints(upright_constraint);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->setTimeout(10.0);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask->add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object");

		pickTask->properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		// TODO TEST WITH THIS
		grasp->setProperty("eef",eef_);

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			/*auto stage = std::make_unique<stages::GeneratePose>("go to grasp pose");
			stage->setPose(graspPose);
			stage->properties().configureInitFrom(Stage::PARENT);*/

			auto stage = std::make_unique<stages::GenerateCustomPose>("Generate Custom Poses");
			stage->setCustomPoses(graspPoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage) );
			wrapper->properties().configureInitFrom(Stage::PARENT, { "group" });
			wrapper->setMaxIKSolutions(10);
			wrapper->setIKFrame(ikFrame_);
			// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(postgrasp);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Attach Object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			grasp->insert(std::move(stage));
		}

		/****************************************************
  		.... *               Allow collision (object support)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			grasp->insert(std::move(stage));
		}
		

		// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  		.... *               Forbid collision (object support)  *
		***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			grasp->insert(std::move(stage));
		}

			// ---------------------- retreat object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.03, 0.25);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}
		pickTask->add(std::move(grasp));
	}

}

void motionPlanning::createPickTaskCustomDual(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup_first,const std::string planGroup_second ,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses_first, std::vector<geometry_msgs::PoseStamped> graspPoses_second)
{
	pickTask->setRobotModel(kinematic_model_);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// Property and variable definitions
	std::string first_pregrasp;
	std::string first_postgrasp;

	std::string second_pregrasp;
	std::string second_postgrasp;

	pickTask->setProperty("object",object);
	if(planGroup_first == "left_arm")
	{
		pickTask->setProperty("eef","left_gripper");
		first_eef_ = "left_gripper";
		first_pregrasp = "left_open";
		first_postgrasp = "left_close";
		first_ikFrame_ = "l_gripper_tool_frame";
	}
	else if(planGroup_first == "right_arm")
	{
		pickTask->setProperty("eef","right_gripper");
		first_eef_ = "right_gripper";
		first_pregrasp = "right_open";
		first_postgrasp = "right_close";
		first_ikFrame_ = "r_gripper_tool_frame";
	}
	

	pickTask->setProperty("group",planGroup_first);

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(PLANNER);

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask->add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand first arm", gripper_planner_);
		stage->setGroup(first_eef_);
		stage->setGoal(first_pregrasp);
		current_state = stage.get();
		pickTask->add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup_first, pipelinePlanner_},{first_eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->setTimeout(10.0);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask->add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object first arm");

		pickTask->properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		// TODO TEST WITH THIS
		grasp->setProperty("eef",first_eef_);

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object first arm", cartesianPlanner_);
			stage->setProperty("group",planGroup_first);
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(first_ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			/*auto stage = std::make_unique<stages::GeneratePose>("go to grasp pose");
			stage->setPose(graspPose);
			stage->properties().configureInitFrom(Stage::PARENT);*/

			auto stage = std::make_unique<stages::GenerateCustomPose>("Generate Custom Poses first arm");
			stage->setCustomPoses(graspPoses_first);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK first arm", std::move(stage) );
			wrapper->setMaxIKSolutions(10);
			wrapper->setProperty("group",planGroup_first);
			wrapper->setIKFrame(first_ikFrame_);
			// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask->getRobotModel()->getJointModelGroup(first_eef_)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand first arm", gripper_planner_);
			stage->setGroup(first_eef_);
			stage->setGoal(first_postgrasp);
			grasp->insert(std::move(stage));
		}

		pickTask->add(std::move(grasp));
	}

	if(planGroup_second == "left_arm")
	{
		pickTask->setProperty("eef","left_gripper");
		second_eef_ = "left_gripper";
		second_pregrasp = "left_open";
		second_postgrasp = "left_close";
		second_ikFrame_ = "l_gripper_tool_frame";
	}
	else if(planGroup_second == "right_arm")
	{
		pickTask->setProperty("eef","right_gripper");
		second_eef_ = "right_gripper";
		second_pregrasp = "right_open";
		second_postgrasp = "right_close";
		second_ikFrame_ = "r_gripper_tool_frame";
	}

	pickTask->setProperty("group",planGroup_second);

	

	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand second arm", gripper_planner_);
		stage->setGroup(second_eef_);
		stage->setGoal(second_pregrasp);
		current_state = stage.get();
		pickTask->add(std::move(stage));
	}


	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup_second, pipelinePlanner_},{second_eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->setTimeout(10.0);
		pickTask->add(std::move(connect));
	}
	
	{
		auto grasp = std::make_unique<SerialContainer>("pick object with second arm");

		// TODO TEST WITH THIS
		grasp->setProperty("eef",second_eef_);

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object second arm", cartesianPlanner_);
			stage->setProperty("group",planGroup_second);
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(second_ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			/*auto stage = std::make_unique<stages::GeneratePose>("go to grasp pose");
			stage->setPose(graspPose);
			stage->properties().configureInitFrom(Stage::PARENT);*/

			auto stage = std::make_unique<stages::GenerateCustomPose>("Generate Custom Poses second arm");
			stage->setCustomPoses(graspPoses_second);
			stage->setProperty("group",planGroup_second);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK second arm", std::move(stage) );
			wrapper->setMaxIKSolutions(10);
			wrapper->setProperty("group",planGroup_second);
			wrapper->setIKFrame(second_ikFrame_);
			wrapper->setProperty("eef",second_eef_);
			// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) second arm");
			stage->allowCollisions(object, pickTask->getRobotModel()->getJointModelGroup(second_eef_)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand second arm", gripper_planner_);
			stage->setGroup(second_eef_);
			stage->setGoal(second_postgrasp);
			grasp->insert(std::move(stage));
		}

		// ---------------------- Attach Object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object to first arm");
			if(planGroup_first == "left_arm")
			{
				stage->attachObject(object, "l_wrist_roll_link");
			}
			else if(planGroup_first == "right_arm")
			{
				stage->attachObject(object, "r_wrist_roll_link");
			}
			grasp->insert(std::move(stage));
		}
		
		pickTask->add(std::move(grasp));
	}
}

 /**
 * \fn void createPickTask(Task &pickTask, const std::string planGroup,const std::string object)
 * \brief Function to create a pick task with grasp pose generator and planGroup (arm)
 *
 * \param pickTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 */
void motionPlanning::createPickTask(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object, const std::string supportId)
{
	pickTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string grasp;

	pickTask->setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask->setProperty("group",planGroup);
		pregrasp = "left_open";
		grasp = "left_close";
		pickTask->setProperty("eef","left_gripper");
	 	eef_ = "left_gripper";
		ikFrame_ = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask->setProperty("group",planGroup);
		pregrasp = "right_open";
		grasp = "right_close";
		pickTask->setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		ikFrame_ = "r_gripper_tool_frame";
	}

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(PLANNER);


	overworld::BoundingBox boundingBoxSrv;
	boundingBoxSrv.request.object_id = object;
	getBoundingBoxSrv_.call(boundingBoxSrv);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask->add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef_,gripper_planner_ }, {planGroup,pipelinePlanner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask->add(std::move(connect));
	}

	{
		auto pick = std::make_unique<SerialContainer>("pick object");
		pickTask->properties().exposeTo(pick->properties(), { "eef", "group"});
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		/****************************************************
 		 ---- *               Approach Object                    *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().set("link", ikFrame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setPreGraspPose(pregrasp);

			stage->setObjectHeight(boundingBoxSrv.response.z);
			stage->setObject(object);
			stage->setAngleDelta(M_PI / 2);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
	
	 		// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)	
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->setIKFrame(ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			pick->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, pickTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			pick->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(grasp);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object support)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);

			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Forbid collision (object support)  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			pick->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		// Add grasp container to task
		pickTask->add(std::move(pick));
	}
}

/**
 * \fn void createPickPlaceTask(std::unique_ptr<moveit::task_constructor::Task>&pickPlaceTask, const std::string planGroup,const std::string object, const std::string supportId, geometry_msgs::PoseStamped placePose)
 * \brief Function to create a pick and place task with grasp pose generator and planGroup (arm)
 *
 * \param pickPlaceTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 * \param supportId support surface on which the object lies on
 * \param placePose Pose to place the object
 */
void motionPlanning::createPickPlaceTask(std::unique_ptr<moveit::task_constructor::Task>&pickPlaceTask, const std::string planGroup,const std::string object, const std::string supportId, geometry_msgs::PoseStamped placePose)
{
	pickPlaceTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string grasp;
	std::string ungrasp;


	if(planGroup == "left_arm" || planGroup == "left_arm_torso")
	{
		pickPlaceTask->setProperty("group",planGroup);
		pregrasp = "left_open";
		ungrasp = "left_open";

		grasp = "left_close";
		pickPlaceTask->setProperty("eef","left_gripper");
		pickPlaceTask->setProperty("ik_frame","l_gripper_tool_frame");

	 	eef_ = "left_gripper";
		ikFrame_ = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm"|| planGroup == "right_arm_torso")
	{
		pickPlaceTask->setProperty("group",planGroup);
		pregrasp = "right_open";
		ungrasp = "right_open";

		grasp = "right_close";
		pickPlaceTask->setProperty("eef","right_gripper");
		pickPlaceTask->setProperty("ik_frame","r_gripper_tool_frame");

		eef_ = "right_gripper";
		ikFrame_ = "r_gripper_tool_frame";
	}

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(PLANNER);

	overworld::BoundingBox boundingBoxSrv;
	boundingBoxSrv.request.object_id = object;
	getBoundingBoxSrv_.call(boundingBoxSrv);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial->setProperty("object",object);
	current_state = initial.get();
	pickPlaceTask->add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->setProperty("group",planGroup);
		connect->setCostTerm(moveit::task_constructor::cost::Clearance{});
		connect->properties().configureInitFrom(Stage::PARENT);
		pickPlaceTask->add(std::move(connect));
	}

	{
		auto pick = std::make_unique<SerialContainer>("pick object");
		pickPlaceTask->properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame"});
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame"});

		/****************************************************
 		 ---- *               Approach Object                    *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().set("link", ikFrame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setPreGraspPose(pregrasp);

			stage->setObjectHeight(boundingBoxSrv.response.z);
			stage->setObject(object);
			stage->setAngleDelta(M_PI / 2);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(100);
			wrapper->setMinSolutionDistance(1.0);
	
	 		// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)	
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->setIKFrame(ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			pick->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, pickPlaceTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			pick->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(grasp);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object support)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);

			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		/****************************************************
  .... *               Forbid collision (object support)  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			pick->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));

		}

		// Add grasp container to task
		pickPlaceTask->add(std::move(pick));
	}

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners =  {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect to place", planners);
		connect->setProperty("group",planGroup);
		connect->setCostTerm(moveit::task_constructor::cost::Clearance{});
		connect->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});
		pickPlaceTask->add(std::move(connect));
	}

	{
		auto place = std::make_unique<SerialContainer>("place object");
		pickPlaceTask->properties().exposeTo(place->properties(), { "eef", "group", "ik_frame"});
		place->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});

		/****************************************************
 		 ---- *               Approach Object                    *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().set("link", ikFrame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->setPose(placePose);
			stage->setObject(object);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("pose IK place", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame_);
			// Fix to avoid getting solution with collision (github.com/ros-planning/moveit_task_constructor/issues/209)
			wrapper->setCostTerm(moveit::task_constructor::cost::Clearance{});
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(ungrasp);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object, ikFrame_);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat from object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.02, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}
		pickPlaceTask->add(std::move(place));
	}
}

 /**
 * \fn void updateWorld(ros::ServiceClient& udwClient)
 * \brief Function to ask ontologenius about object id/meshes that are on the table then ask underworld their positions, and add them to planning scene
 *
 * \param udwClient Handle on the underworld service to get poses of the object in the scene.
 *
 * \return 0 if everything went fine,
 * 		   1 if failed to get transform between map and base_footprint
 * 		   2 if failed to get meshes from ontologenius
 * 		   3 if failed to get Ids from Underworld service
 */
int motionPlanning::updateWorld(ros::ServiceClient& udwClient)
{
	ROS_DEBUG_STREAM("===============[UPDATE OF THE WORLD INCOMING]==================");
	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
	shapes::Mesh* m;
	std::string meshURI;
	std::vector<std::string> meshTemp;

	std::string dimensionStringRaw;
	std::vector<std::string> dimensionTemp;
	std::vector<float> objDimensions;

	moveit_msgs::CollisionObject collisionObj;

	moveit_msgs::CollisionObject throwBox_left;
	moveit_msgs::CollisionObject throwBox_right;

	geometry_msgs::PoseStamped colliObjPosetransformed;
	geometry_msgs::PoseStamped colliObjPoseUntransformed;

	std::vector<std::string> objIds;
	std::vector<std::string> furnitureIds;

	pr2_motion_tasks_msgs::GetPose srv;

	// Delete all objects from the scene to avoid artifacts.
	std::vector<std::string> objToDelete = planning_scene_interface_.getKnownObjectNames();
	std::map< std::string,moveit_msgs::AttachedCollisionObject> knownAttachedObj = planning_scene_interface_.getAttachedObjects();
	std::map< std::string,moveit_msgs::AttachedCollisionObject>::iterator it;
	for (int i =0; i < objToDelete.size(); i++)
	{
		// Only delete not attached object
		it = knownAttachedObj.find(objToDelete[i]);
		if (it == knownAttachedObj.end())
		{
			collisionObj.id = objToDelete[i];
			collisionObj.operation = collisionObj.REMOVE;
			planning_scene_interface_.applyCollisionObject(collisionObj);
		}
	}

	furnitureIds = onto_.individuals.getType("Furniture");
	ROS_INFO_STREAM("--===============[There is " << furnitureIds.size() << " Furnitures in the scene" << "]==================--");

	// For all furniture, get all the object that are on them
	for (int j=0; j < furnitureIds.size(); j++)
	{
		ROS_INFO_STREAM("##===============[Furniture is " << furnitureIds[j] << "]==================##");
	
		srv.request.ids = furnitureIds;
		if (udwClient.call(srv))
		{
			// Clear vector to be able to reuse it
			collisionObj.meshes.clear();
			collisionObj.mesh_poses.clear();

			collisionObj.primitives.clear();
			collisionObj.primitive_poses.clear();

			// Fill in mesh URI (ask ontology or get it from the cache)
			//Verify if frame_id isn't empty
			// UWDS publish with frame_id as /map so transform to base_footprint
			if(!srv.response.poses[j].header.frame_id.empty())
			{
				// Check object is already known and if a mesh is already in the database
				if (objMeshMap_.find(furnitureIds[j]) != objMeshMap_.end())
				{
					// Mesh URI is already known so get it from the map
					m = shapes::createMeshFromResource(objMeshMap_.at(furnitureIds[j]));

					shapes::constructMsgFromShape(m, mesh_msg);
					mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

					// Add the mesh to the Collision object message
					collisionObj.meshes.push_back(mesh);

					if(srv.response.poses[j].header.frame_id != "/base_footprint")
					{
						// Transform pose given by UWDS from map to basefootprint
						colliObjPoseUntransformed.pose = srv.response.poses[j].pose;
						colliObjPoseUntransformed.header.frame_id = srv.response.poses[j].header.frame_id;

						getPoseIntoBasefootprint(colliObjPoseUntransformed,colliObjPosetransformed);
												
						collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
					}
					else
					{
						collisionObj.mesh_poses.push_back(srv.response.poses[j].pose);
					}
				}
				// Check if object is a mesh
				else if (onto_.individuals.getOn(furnitureIds[j],"hasMesh").size() > 0)
				{
					// Ask ontologenius for mesh URI
					meshTemp = onto_.individuals.getOn(furnitureIds[j],"hasMesh");
					if(meshTemp.size() > 0)
					{
						meshURI = meshTemp[0];

						size_t pos = meshURI.find("string#");
						if (pos != std::string::npos)
						{
							// If found then erase it from string
							meshURI.erase(pos, std::string("string#").length());
						}

							ROS_INFO_STREAM("furnitureId is [" << furnitureIds[j] << "]" );
							ROS_INFO_STREAM("MESH_URI is [" << meshURI << "]" );

						m = shapes::createMeshFromResource(meshURI);

						// And add it to the map
						objMeshMap_.insert(std::make_pair<std::string,std::string>((std::string)furnitureIds[j],(std::string)meshURI));

						shapes::constructMsgFromShape(m, mesh_msg);
						mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

						// Add the mesh to the Collision object message
						collisionObj.meshes.push_back(mesh);

						if(srv.response.poses[j].header.frame_id != "/base_footprint")
						{
							// Transform pose given by UWDS from map to basefootprint
							colliObjPoseUntransformed.pose = srv.response.poses[j].pose;
							colliObjPoseUntransformed.header.frame_id = srv.response.poses[j].header.frame_id;
							getPoseIntoBasefootprint(colliObjPoseUntransformed,colliObjPosetransformed);
													
							collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
						}
						else
						{
							collisionObj.mesh_poses.push_back(srv.response.poses[j].pose);
						}
					}
				}
				else
				{
					ROS_ERROR_STREAM("No mesh specified for furniture with id [" << furnitureIds[j] << "]");
					continue;
				}

				// Set object id
				collisionObj.id = furnitureIds[j];

				// Set frame_id to "base_footprint" as it has been transformed
				collisionObj.header.frame_id = "base_footprint";

				collisionObj.operation = collisionObj.ADD;
			
				// Add synchronously the collision object to planning scene (wait for it to be added before continuing)
				planning_scene_interface_.applyCollisionObject(collisionObj);
				ROS_INFO_STREAM("Successfully added to scene furniture with ID [" + furnitureIds[j] + "]");
			}

			ROS_INFO_STREAM("Now adding objects that are on furniture with ID [" + furnitureIds[j] + "]");

			objIds = onto_.individuals.getOn(furnitureIds[j],"isBelow");

			ROS_INFO_STREAM("===============[There is " << objIds.size() << " objects on top of it]==================");
		}
		else
		{
			ROS_ERROR_STREAM("===============[Call of UWDS service to get pose failed for furniture !]==================");
			return -6;
		}

		// Ask underworld about poses of these ids
		srv.request.ids = objIds;
		if (udwClient.call(srv))
		{
			for (int i=0; i < objIds.size(); i++)
			{
				// Clear vector to be able to reuse it
				collisionObj.meshes.clear();
				collisionObj.mesh_poses.clear();

				collisionObj.primitives.clear();
				collisionObj.primitive_poses.clear();

				// Fill in mesh URI (ask ontology or get it from the cache)
				//Verify if frame_id isn't empty
				// UWDS publish with frame_id as /map so transform to base_footprint
				if(!srv.response.poses[i].header.frame_id.empty())
				{
					
					// Check object is already known and if a mesh is already in the database
					if (objMeshMap_.find(objIds[i]) != objMeshMap_.end())
					{
						// Mesh URI is already known so get it from the map
						m = shapes::createMeshFromResource(objMeshMap_.at(objIds[i]));

						shapes::constructMsgFromShape(m, mesh_msg);
						mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

						// Add the mesh to the Collision object message
						collisionObj.meshes.push_back(mesh);

						if(srv.response.poses[i].header.frame_id != "/base_footprint")
						{
							// Transform pose given by UWDS from map to basefootprint
							colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
							colliObjPoseUntransformed.header.frame_id = srv.response.poses[j].header.frame_id;

							getPoseIntoBasefootprint(colliObjPoseUntransformed,colliObjPosetransformed);
													
							collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
						}
						else
						{
							collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
						}
					}
					// Check if object is a mesh
					else if (onto_.individuals.getOn(objIds[i],"hasMesh").size() > 0)
					{
						// Ask ontologenius for mesh URI
						meshTemp = onto_.individuals.getOn(objIds[i],"hasMesh");
						if(meshTemp.size() > 0)
						{
							meshURI = meshTemp[0];

							size_t pos = meshURI.find("string#");
							if (pos != std::string::npos)
							{
								// If found then erase it from string
								meshURI.erase(pos, std::string("string#").length());
							}

								ROS_INFO_STREAM("ObjId is [" << objIds[i] << "]" );
								ROS_INFO_STREAM("MESH_URI is [" << meshURI << "]" );

							m = shapes::createMeshFromResource(meshURI);

							// And add it to the map
							objMeshMap_.insert(std::make_pair<std::string,std::string>((std::string)objIds[i],(std::string)meshURI));

							shapes::constructMsgFromShape(m, mesh_msg);
							mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

							// Add the mesh to the Collision object message
							collisionObj.meshes.push_back(mesh);

							if(srv.response.poses[i].header.frame_id != "/base_footprint")
							{
								// Transform pose given by UWDS from map to basefootprint
								colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
								colliObjPoseUntransformed.header.frame_id = srv.response.poses[j].header.frame_id;

								getPoseIntoBasefootprint(colliObjPoseUntransformed,colliObjPosetransformed);
														
								collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
							}
							else
							{
								collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
							}
						}
					}
					// Check if object has shape defined in ontology
					else if (onto_.individuals.getOn(objIds[i],"hasShape").size() > 0)
					{
						ROS_WARN_STREAM("Error while updating the world, no meshes were returned by Ontologenius...");	
						ROS_WARN_STREAM("Asking for dimensions if objects comes from Robosherlock...");

						collisionObj.primitives.resize(1);

						if(srv.response.poses[i].header.frame_id != "/base_footprint")
						{
							// Transform pose given by UWDS from map to basefootprint
							colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
							colliObjPoseUntransformed.header.frame_id = srv.response.poses[j].header.frame_id;

							getPoseIntoBasefootprint(colliObjPoseUntransformed,colliObjPosetransformed);
													
							collisionObj.primitive_poses.push_back(colliObjPosetransformed.pose);
						}
						else
						{
							collisionObj.primitive_poses.push_back(srv.response.poses[i].pose);
						}

						// Set the type of object 
						if(onto_.individuals.getOn(objIds[i],"hasShape")[0] == "box")
						{
							ROS_WARN_STREAM("It's a box !!");

							collisionObj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
							
							collisionObj.primitives[0].dimensions.resize(3);
							sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionX")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
							sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionY")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
							sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionZ")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
							
							//ROS_INFO_STREAM("Dimensions of ID [" << objIds[i] << "] ---- X : " << collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
							//ROS_INFO_STREAM("Dimensions of ID [" << objIds[i] << "] ---- Y : " << collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
							//ROS_INFO_STREAM("Dimensions of ID [" << objIds[i] << "] ---- Z : " << collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);																ROS_INFO_STREAM("Dimensions of ID [" << objIds[i] << "] ---- Z : " << collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);								
						}
						else if (onto_.individuals.getOn(objIds[i],"hasShape")[0] == "round")
						{
							ROS_WARN_STREAM("It's a cylinder !!");

							collisionObj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
							
							collisionObj.primitives[0].dimensions.resize(2);
							sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionY")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);
							sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionZ")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);

							collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/=2.0;
						}
						else
						{
							ROS_ERROR("No shape specified for Robosherlock objects....");
							continue;
						}	
					}
					else
					{
						ROS_ERROR_STREAM("No shape, no mesh specified for object with id [" << objIds[i] << "]");
						continue;
					}
												
					// Set object id
					collisionObj.id = objIds[i];

					collisionObj.operation = collisionObj.ADD;
				
					// Set frame_id to "base_footprint" as it has been transformed
					collisionObj.header.frame_id = "base_footprint";

					// Add synchronously the collision object to planning scene (wait for it to be added before continuing)
					planning_scene_interface_.applyCollisionObject(collisionObj);
					ROS_INFO_STREAM("Successfully added to scene object with ID [" + collisionObj.id + "]");
				}
				else
				{
					ROS_WARN_STREAM("Error while updating the world, frame_id is empty...");
				}
				ROS_INFO_STREAM("----------------------------------------------------------------------------");
			}
		}
		else
		{
			ROS_ERROR_STREAM("===============[Call of UWDS service to get pose failed for objects !]==================");
			return -6 ;
		}
	
		ROS_INFO_STREAM("###########################################################################################");	
	}
}

 /**
 * \fn void taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat)
 * \brief Callback that is used to send back progress value whem planning for a task
 *
 * \param taskStat Pointer to get the number of stages already solved
 */
void motionPlanning::taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat)
{
	pr2_motion_tasks_msgs::planFeedback planFeedback;
	int progressValue=0;

	progressValue = (int)(((double)taskStat->stages[0].solved.size()/NUMBER_OF_MAX_SOLUTION)*100.0);

	planFeedback.status = progressValue;
	planServer_->publishFeedback(planFeedback);
}

 /**
 * \fn void solutionCallback(const moveit_task_constructor_msgs::SolutionConstPtr& solution, int& cost)
 * \brief Callback that is used to ask Moveit Task Constructor about the cost of the choosen solution (least cost for now)
 *
 * \param solution Solutions lists found during the last planning
 * \param cost Cost that will be sent back to supervisor
 */
void solutionCallback(const moveit_task_constructor_msgs::SolutionConstPtr& solution, int& cost)
{
	/*for(int i=0; i < solution->sub_solution.size(); i++)
	{
		 ROS_ERROR_STREAM("COST RECEIVED[" << i << "] : " << solution->sub_solution[i].info.cost);
	}*/
	cost = solution->sub_solution[0].info.cost;
}

 /**
 * \fn void getPoseIntoBasefootprint(const geometry_msgs::PoseStampedConstPtr& in_pose, geometry_msgs::PoseStamped* out_pose)
 * \brief Function to get input pose into base_footprint frame (to get same orientation everytime)
 *
 * \param in_pose Input pose unstransformed
 * \param out_pose Pose transformed into base_footprint frame
 */
void motionPlanning::getPoseIntoBasefootprint(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped& out_pose)
{
	geometry_msgs::TransformStamped transform;

	try
	{
		if (in_pose.header.frame_id[0] == '/')
			in_pose.header.frame_id = in_pose.header.frame_id.substr(1);

		transform = tfBuffer_.lookupTransform("base_footprint",in_pose.header.frame_id, ros::Time(0),ros::Duration(5.0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		return ;
	}

	tf2::doTransform(in_pose,out_pose,transform);
}

 /**
 * \fn void planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& udwClient)
 * \brief Callback that is called when supervisor ask for a plan
 *
 * \param goal Goal sent by supervisor. Contains action to be planned (pick, place, move), planGroup to be used if moving, object if pick, box if place
 * \param udwClient Service handle to pass on to the update world function
 */
void motionPlanning::planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& udwClient)
{
 	pr2_motion_tasks_msgs::planResult planResult;

	std::vector<geometry_msgs::PoseStamped> customPoses;
    geometry_msgs::PoseStamped customPose;

	geometry_msgs::PoseStamped transformedPose;

	std::vector<std::string> supportSurfaceId;
	std::vector<std::string> boxesIds;
	std::vector<std::string> objInBoxIds;
	moveit_msgs::CollisionObject collisionObj;


	int updateWorldResult = 0;

	std::string taskName;

	if((goal->action == "pick") || (goal->action == "pick_dt") ||  (goal->action == "pickPlace") || (goal->action == "pickAuto") ||  (goal->action == "pickDual") || (goal->action == "updateWorld") )
	{
		updateWorldResult = updateWorld(udwClient);

		// Ask the box in which the cube is 
		if (goal->action != "updateWorld")
		{
			// Verify if it's in a box (for director task only)
			supportSurfaceId = onto_.individuals.getOn(goal->objId,"isIn");
			if (supportSurfaceId.size() == 0)
			{
				// If not in a box, verify if it's on something like a table (general use case)
				supportSurfaceId = onto_.individuals.getOn(goal->objId,"isOnTopOf");
				if (supportSurfaceId.size() == 0)
				{
					planResult.error_code = -1;
					ROS_ERROR_STREAM("[Anti-gravity generator enabled]" << goal->objId << " isn't in any box or on top of any surface.");
					planServer_->setAborted(planResult);
					return;
				}
			}
		}

		if(updateWorldResult == 1)
		{
			planResult.error_code = -4;
			planServer_->setAborted(planResult);
			return;
		}
		else if(updateWorldResult == 2)
		{
			planResult.error_code = -5;
			planServer_->setAborted(planResult);
			return;
		}
		else if(updateWorldResult == 3)
		{
			planResult.error_code = -6;
			planServer_->setAborted(planResult);
			return;
		}
		else if (goal->action == "updateWorld")
		{
			planResult.error_code = 1;
			planServer_->setSucceeded(planResult);
			return;
		}

		if((goal->action == "pickDual"))
		{
			// check if objId is a box. If we want to pick a box, we need to delete temporarily the object that are inside
			if(onto_.individuals.isA(goal->objId,"Box"))
			{
				// the object we are trying to pick is a box, delete all object that are inside it
				objInBoxIds = onto_.individuals.getFrom("isIn",goal->objId);

				if(objInBoxIds.size() != 0)
				{
					ROS_ERROR_STREAM("--===============[There is " << objInBoxIds.size() << " object in " << goal->objId << " . Deleting them before planning pick]==================--");
					
					for (int i =0; i < objInBoxIds.size(); i++)
					{
						ROS_WARN_STREAM("--=================== DELETING [" << objInBoxIds[i] << "] from the scene ===================--");

						collisionObj.id = objInBoxIds[i];
						collisionObj.operation = collisionObj.REMOVE;
						planning_scene_interface_.applyCollisionObject(collisionObj);
					}
				}
			} 
		}
	}

	if(goal->planGroup.empty())
	{
		std::vector<std::string> objIds;
		objIds.push_back(goal->objId);
		if(planning_scene_interface_.getObjectPoses(objIds).find(goal->objId)->second.position.y > 0)
		{
			taskArmGroup_ = "left_arm";
		}
		else
		{
			taskArmGroup_ = "right_arm";
		}
	}
	else
	{
		taskArmGroup_ = goal->planGroup;
	}

	// reset to avoid problem on introspection
	lastPlannedTask_.reset();

	taskObjId_ = goal->objId;

	//====== PICK ======//
	if(goal->action == "pick")
	{

		getPoseIntoBasefootprint(goal->pose,transformedPose);

		transformedPose.header.frame_id = "/base_footprint";

		debug_pose_pub_.publish(transformedPose);

		customPoses.push_back(transformedPose);

		taskName = goal->action + "_" + goal->objId;

		ROS_ERROR_STREAM("Support surface for pick is [" << supportSurfaceId[0] << "]");

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);
		createPickTaskCustom(lastPlannedTask_,taskArmGroup_,goal->objId,supportSurfaceId[0], customPoses);
	}
	else if(goal->action == "pickPlace")
	{

		getPoseIntoBasefootprint(goal->pose,customPose);

		customPose.header.frame_id = "/base_footprint";

		taskName = goal->action + "_" + goal->objId;

		ROS_ERROR_STREAM("Support surface for pick is [" << supportSurfaceId[0] << "]");

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		createPickPlaceTask(lastPlannedTask_,taskArmGroup_,goal->objId,supportSurfaceId[0], customPose);
	}
	else if(goal->action == "pick_dt")
	{
		customPose.header.frame_id = goal->objId;
		customPose.pose.position.x = -0.02;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.0;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 0.0;
		customPose.pose.orientation.w = 1.0;
		customPoses.push_back(customPose);


		customPose.header.frame_id = goal->objId;
		customPose.pose.position.x = 0.02;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.0;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 1.0;
		customPose.pose.orientation.w = 0.0;
		customPoses.push_back(customPose);

		taskName = goal->action + "_" + goal->objId;

		ROS_ERROR_STREAM("Support surface for pick is [" << supportSurfaceId[0] << "]");

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		createPickTaskCustom(lastPlannedTask_,taskArmGroup_,goal->objId,supportSurfaceId[0], customPoses);
	}
	else if(goal->action == "pickDual")
	{
		customPoses.push_back(goal->pose);
		std::vector<geometry_msgs::PoseStamped> customPoses_right;

	    geometry_msgs::PoseStamped customPose_right = goal->pose;
		customPose_right.pose.position.x *= -1;
		customPoses_right.push_back(customPose_right);

		std::string delim = "+";
		std::string armGroup_left = "";
		std::string armGroup_right = "";

		auto start = 0U;
		auto end = taskArmGroup_.find(delim);
		
		armGroup_left = taskArmGroup_.substr(start, end - start);
		start = end + delim.length();
		end = taskArmGroup_.find(delim, start);

		armGroup_right = taskArmGroup_.substr(start, end - start);
		start = end + delim.length();
		end = taskArmGroup_.find(delim, start);
	
		taskName = goal->action + "_" + goal->objId + "with" + armGroup_left + "and" + armGroup_right;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);
		createPickTaskCustomDual(lastPlannedTask_,armGroup_left, armGroup_right,goal->objId,supportSurfaceId[0], customPoses,customPoses_right);
	}
	else if(goal->action == "pickAuto")
	{
		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);
		createPickTask(lastPlannedTask_,taskArmGroup_,goal->objId,supportSurfaceId[0]);
	}
	//====== PLACE ======//
	else if(goal->action == "place")
	{

		getPoseIntoBasefootprint(goal->pose,transformedPose);

		transformedPose.header.frame_id = "/base_footprint";
		transformedPose.pose.position.z += 0.05;
	
		debug_pose_pub_.publish(transformedPose);


		customPoses.push_back(transformedPose);

		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);
		createPlaceTask(lastPlannedTask_, taskArmGroup_, goal->objId, customPoses);
	}
	else if(goal->action == "place_dt")
	{
		customPose.header.frame_id = goal->pose.header.frame_id;
		customPose.pose.position.x = -0.02;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = -0.035;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 0.0;
		customPose.pose.orientation.w = 1.0;
		customPoses.push_back(customPose);


		customPose.header.frame_id = goal->pose.header.frame_id;
		customPose.pose.position.x = 0.02;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = -0.035;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 1.0;
		customPose.pose.orientation.w = 0.0;
		customPoses.push_back(customPose);

		taskName = goal->action + "_" + goal->objId + "_in_" + goal->pose.header.frame_id;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);
		createPlaceTask(lastPlannedTask_, taskArmGroup_, goal->objId, customPoses);
	}
	//====== MOVE ======//
	else if (goal->action == "move")
	{
		taskName = goal->action + "_" + goal->planGroup;
		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);
		lastPlannedTask_->setName(taskName);

		if(goal->predefined_pose_id.empty())
		{
			createMoveTask(lastPlannedTask_, taskArmGroup_,goal->pose);
		}
		else
		{
			if((goal->predefined_pose_id == "left_arm_home") && (taskArmGroup_ == ""))
			{
				taskArmGroup_ = "left_arm";
			}
			else if((goal->predefined_pose_id == "right_arm_home") && (taskArmGroup_ == ""))
			{
				taskArmGroup_ = "right_arm";
			}
			createMovePredefinedTask(lastPlannedTask_, taskArmGroup_,goal->predefined_pose_id);
		}
	}
	//====== DROP ======//
	else if (goal->action == "drop")
	{
		taskName = goal->action + "_" + goal->planGroup;
		// Create Task
		lastPlannedTask_ = std::make_unique<Task>(taskName);

		lastPlannedTask_->setName(taskName);

		createDropTask(lastPlannedTask_, taskArmGroup_,goal->objId, goal->pose.header.frame_id);
	}
	else
	{
		ROS_ERROR_STREAM("Unknown action provided. Available action are pick, pick_dt, pickDual, move, drop, place, place_dt");
		// TODO handle this case
		return;
	}

	// Create Thread to handle the feedback process 
	std::string statTopic = "/pr2_tasks_node/" + taskName + "/statistics";
	ros::Subscriber sub = nh_.subscribe<moveit_task_constructor_msgs::TaskStatistics>(statTopic, 10, &motionPlanning::taskStatisticCallback, this);

	try
	{
		ROS_INFO_STREAM("Beginning plan of task [" << taskName << "] !");

		if(lastPlannedTask_->plan(NUMBER_OF_MAX_SOLUTION) && !planServer_->isPreemptRequested())
		{
			ROS_INFO_STREAM("Planning of task [" << taskName << "] SUCCEEDED !");

			planResult.error_code = 1;
			planResult.cost = lastPlannedTask_->solutions().front()->cost();
			planResult.armUsed = taskArmGroup_;
			pr2_motion_tasks_msgs::planFeedback planFeedback;
			planFeedback.status = 100;
			planServer_->publishFeedback(planFeedback);
			planServer_->setSucceeded(planResult);
		}
		else
		{
			planResult.error_code = -1;
			
			if(planServer_->isPreemptRequested())
			{
				ROS_WARN_STREAM("Planning of task [" << taskName << "] PREEMPTED !");
				planServer_->setPreempted(planResult);
			}
			else
			{
				ROS_WARN_STREAM("Planning of task [" << taskName << "] ABORTED !");
				planServer_->setAborted(planResult);
			}
		}
	}
	catch (const InitStageException& e)
	{
    // TODO Handle this state
		ROS_ERROR_STREAM(e);
	}

}

void feedbackCb(const moveit_task_constructor_msgs::ExecuteTaskSolutionFeedbackConstPtr& feedback)
{
  ROS_ERROR_STREAM("Got Feedback ID  " <<  feedback->sub_id);
  ROS_ERROR_STREAM("Got Feedback Number  " <<  feedback->sub_no);
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const moveit_task_constructor_msgs::ExecuteTaskSolutionResultConstPtr& result, bool& doneFlag)
{
  ROS_INFO_STREAM("Finished in state : " << state.toString().c_str());
  ROS_INFO_STREAM("RESULT: " << result->error_code);

  doneFlag = true;
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}



 /**
 * \fn void executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal,  ros::Publisher factsPublisher)
 * \brief Callback that is called when supervisor ask to execute last planned task
 *
 * \param goal Goal sent by supervisor. Void
 */
void motionPlanning::executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal)
{
	pr2_motion_tasks_msgs::executeFeedback executeFeedback;
  	pr2_motion_tasks_msgs::executeResult executeResult;

	moveit_msgs::AttachedCollisionObject collisionAttObj;
	moveit_msgs::CollisionObject collisionObj;

	bool doneFlag = false;

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> executeTask("execute_task_solution", true);
	executeTask.waitForServer();

	// Verify that task had solutions
	if(lastPlannedTask_->solutions().size() > 0)
	{
    // TODO Check that this works
		//ROS_INFO_STREAM("Executing solution trajectory of " << lastPlannedTask_->ns());

		// Fill the solution message
		lastPlannedTask_->solutions().front()->fillMessage(execute_goal.solution);

		ROS_INFO_STREAM("Sending goal to execute the previous task ("<< lastPlannedTask_->name() << ")");

		executeTask.sendGoal(execute_goal, boost::bind(&doneCb,_1,_2,boost::ref(doneFlag)), &activeCb, &feedbackCb);
		executeFeedback.action_start = ros::Time::now();
		int dummyProgress = 0;
		ros::Rate loop_rate(1);
		while(!doneFlag)
		{
			executeFeedback.status = dummyProgress;
			dummyProgress++;
			executeServer_->publishFeedback(executeFeedback);
			if(executeServer_->isPreemptRequested())
			{
				executeTask.cancelGoal();
			}

			loop_rate.sleep();
		}

		moveit_msgs::MoveItErrorCodes execute_result = executeTask.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ROS_ERROR_STREAM("Task execution failed and returned: " << executeTask.getState().toString());

			executeResult.error_code = -2;

			if(executeServer_->isPreemptRequested())
			{
				executeServer_->setPreempted(executeResult);
			}
			else
			{
				executeServer_->setAborted(executeResult);
			}
			
		}
		else
		{
			// If task name was a pick or place, check if something has been really attached/detached
			// If not delete object attached from moveit world
			if(lastPlannedTask_->id().find("pick") != std::string::npos)
			{
				if(onto_.individuals.getOn("pr2_robot","hasInLeftHand").size() > 0 && taskArmGroup_ == "left_arm")
				{
					executeResult.error_code = 1;
					executeResult.action_end = ros::Time::now();
					executeServer_->setSucceeded(executeResult);
					ROS_INFO_STREAM("Task execution succeeded and returned: " << executeTask.getState().toString());
				}
				else if(onto_.individuals.getOn("pr2_robot","hasInRightHand").size() > 0 && taskArmGroup_ == "right_arm")
				{
					executeResult.error_code = 1;
					executeResult.action_end = ros::Time::now();
					executeServer_->setSucceeded(executeResult);
					ROS_INFO_STREAM("Task execution succeeded and returned: " << executeTask.getState().toString());
				}
				else
				{
					//Detach object from gripper 
					collisionAttObj.object.id = taskObjId_;
					collisionAttObj.link_name = "";
					collisionAttObj.object.operation = collisionAttObj.object.REMOVE;
					planning_scene_interface_.applyAttachedCollisionObject(collisionAttObj);	

					// Delete object from moveit world
					collisionObj.id = taskObjId_;
					collisionObj.operation = collisionObj.REMOVE;
					planning_scene_interface_.applyCollisionObject(collisionObj);					
			
					ROS_ERROR_STREAM("Task execution succeeded but SA returned that object isn't held by gripper");
					ROS_ERROR_STREAM("Obj [" << taskObjId_ << "] has been deleted from moveit world");
					executeResult.error_code = -2;
					executeServer_->setAborted(executeResult);
				}
			}	
			else
			{
				executeResult.error_code = 1;
				executeResult.action_end = ros::Time::now();
				executeServer_->setSucceeded(executeResult);
				ROS_INFO_STREAM("Task execution succeeded and returned: " << executeTask.getState().toString());
			}
		}
	}
	else
	{
		ROS_WARN_STREAM("Execution of task failed because the last planned task had no solution !");
		executeResult.error_code = -3;
		executeServer_->setAborted(executeResult);
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_tasks_node");
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::NodeHandle nh("~");
	ros::Rate r(10); // 10 hz+

	motionPlanning pr2Motion(nh);

	ROS_ERROR("STARTED ACTION SERVS");

	while(ros::ok());

	return 0;
}

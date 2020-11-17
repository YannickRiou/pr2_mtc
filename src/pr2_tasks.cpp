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
motionPlanning::motionPlanning()
  : onto_(&nh_),
	robot_model_loader_("robot_description"),
	transformListenner_(tfBuffer_)
{
	// Close ontology
	onto_.close();

	// Set to verbose when debugging
	//onto_.verbose(true);

	// Load the common kinematic model of the robot that will be used when creating a task with MTC
	kinematic_model_ = robot_model_loader_.getModel();

	// Create the common cartesian planner that will be used for translation movement when creating task with MTC
	cartesianPlanner_ = std::make_shared<solvers::CartesianPath>();
	cartesianPlanner_->setProperty("jump_threshold", 0.00);

	// Create the common pipeline planner (by default RRTConnect) that will be used when creating a task with MTC
	pipelinePlanner_ = std::make_shared<solvers::PipelinePlanner>();
	pipelinePlanner_->setPlannerId("RRTConnect");

	// Define the distance between two waypoint in trajectory.
	// Large value might lead to a trajectory going through collision objects
	// Small value will increase computing time
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.01);

	// Create the common planner for gripper open/close movement when creating task with MTC
	gripper_planner_ = std::make_shared<solvers::JointInterpolationPlanner>();
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
void motionPlanning::createPlaceTask(Task &placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses)
{
	placeTask.setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string ungrasp;

	placeTask.setProperty("object",object);

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
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","left_gripper");
	    eef_ = "left_gripper";
		ikFrame_ = "l_gripper_tool_frame";
		ungrasp = "left_open";

		c.link_name= "l_gripper_tool_frame";

	}
	else if(planGroup == "right_arm")
	{
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		ikFrame_ = "r_gripper_tool_frame";
		ungrasp = "right_open";

		c.link_name= "r_gripper_tool_frame";

	}

	// Increase precision for place to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);

	geometry_msgs::PoseStamped ik;
	ik.header.frame_id= ikFrame_;
	ik.pose.position.x=  0;
	ik.pose.position.y=  0;
	ik.pose.position.z=  0;
	ik.pose.orientation.x=  0;
	ik.pose.orientation.y=  0;
	ik.pose.orientation.z=  0;
	ik.pose.orientation.w=  1;
	placeTask.setProperty("ik_frame",ik);

	pipelinePlanner_->setPlannerId("RRTConnect");

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	// Copy properties defined for placeTask to initial stages (then it will be possible to get them into other stages)
	placeTask.properties().exposeTo(initial->properties(), { "eef", "group","ik_frame" });
	placeTask.add(std::move(initial));

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		connect->setPathConstraints(upright_constraint);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		placeTask.add(std::move(connect));
	}

	{
		auto place = std::make_unique<SerialContainer>("place object");

		placeTask.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
		place->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});

		// Temporary set an approach stage to avoid solutions with collisions
		{
			std::vector<geometry_msgs::PoseStamped> approachPlacePoses;
			geometry_msgs::PoseStamped approach;
			approach = placePoses[0];
			approach.pose.position.x = approach.pose.position.x-0.20;
			approach.pose.position.z = approach.pose.position.z-0.035;
			approachPlacePoses.push_back(approach);

			approach = placePoses[1];
			approach.pose.position.x = approach.pose.position.x+0.20;
			approach.pose.position.z = approach.pose.position.z-0.035;
			approachPlacePoses.push_back(approach);

			auto stage = std::make_unique<stages::GenerateCustomPose>("approach to pose");
			stage->setCustomPoses(approachPlacePoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);
			current_state = stage.get();

			auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("place object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.12, 0.15);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
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
			stage->setMinMaxDistance(0.15, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}
		placeTask.add(std::move(place));

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
void motionPlanning::createMoveTask(Task &moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
{
	moveTask.setRobotModel(kinematic_model_);

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
	moveTask.add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		moveTask.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef_);
		stage->setPose(moveToPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(ikFrame_);
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef_);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		moveTask.add(std::move(wrapper));
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
void motionPlanning::createMovePredefinedTask(Task &moveTask, const std::string planGroup,const std::string pose_id)
{
	moveTask.setRobotModel(kinematic_model_);

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);
	pipelinePlanner_->setPlannerId("RRTConnect");

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	moveTask.add(std::move(initial));

	{
		auto stage = std::make_unique<stages::MoveTo>("Move to pre-defined pose", pipelinePlanner_);
		stage->setGroup(planGroup);
		stage->setGoal(pose_id);
		moveTask.add(std::move(stage));
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
void motionPlanning::createDropTask(Task &dropTask, const std::string planGroup,const std::string object)
{
	dropTask.setRobotModel(kinematic_model_);

	std::string homePoseId;
	std::string ungrasp;

	geometry_msgs::PoseStamped dropPose;



	if(planGroup == "left_arm")
	{
		ikFrame_ = "l_gripper_tool_frame";
		eef_ = "left_gripper";
		homePoseId = "left_arm_home";
		dropPose.header.frame_id = "throw_box_left";

		ungrasp = "left_open";
	}
	else if(planGroup == "right_arm")
	{
		ikFrame_ = "r_gripper_tool_frame";
		eef_ = "right_gripper";
		homePoseId = "right_arm_home";
		dropPose.header.frame_id = "throw_box_right";
		ungrasp = "right_open";
	}

	dropPose.pose.position.x = 0.0;
	dropPose.pose.position.y = 0.0;
	dropPose.pose.position.z = 0.7;
	dropPose.pose.orientation.x = 0.0;
	dropPose.pose.orientation.y = 0.707;
	dropPose.pose.orientation.z = 0.0;
	dropPose.pose.orientation.w = 0.707;

	// Increase precision for drop to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);

	pipelinePlanner_->setPlannerId("TRRT");


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	dropTask.add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->properties().configureInitFrom(Stage::PARENT);
		dropTask.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef_);
		stage->setPose(dropPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(ikFrame_);
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef_);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		dropTask.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
		stage->setGroup(eef_);
		stage->setGoal(ungrasp);
		dropTask.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, ikFrame_);
		dropTask.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("Go to home", pipelinePlanner_);
		stage->setGroup(planGroup);
		stage->setGoal(homePoseId);
		dropTask.add(std::move(stage));
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
void motionPlanning::createPickTaskCustom(Task &pickTask, const std::string planGroup,const std::string object, std::vector<geometry_msgs::PoseStamped> graspPoses)
{
	pickTask.setRobotModel(kinematic_model_);

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

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","left_gripper");
	    eef_ = "left_gripper";
		pregrasp = "left_open";
		postgrasp = "left_close";
		ikFrame_ = "l_gripper_tool_frame";

		c.link_name= "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		pregrasp = "right_open";
		postgrasp = "right_close";
		ikFrame_ = "r_gripper_tool_frame";

		c.link_name= "r_gripper_tool_frame";
	}

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId("TRRT");

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", gripper_planner_);
		stage->setGroup(eef_);
		stage->setGoal(pregrasp);
		current_state = stage.get();
		pickTask.add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->setPathConstraints(upright_constraint);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->setTimeout(10.0);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask.add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object");

		pickTask.properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		// TODO TEST WITH THIS
		grasp->setProperty("eef",eef_);

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.02, 0.20);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
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
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame_);
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask.getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),true);
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

			// ---------------------- retreat object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.15, 0.20);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}
		pickTask.add(std::move(grasp));
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
void motionPlanning::createPickTask(Task &pickTask, const std::string planGroup,const std::string object)
{
	pickTask.setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string grasp;

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "left_open";
		grasp = "left_close";
		pickTask.setProperty("eef","left_gripper");
	 	eef_ = "left_gripper";
		ikFrame_ = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "right_open";
		grasp = "right_close";
		pickTask.setProperty("eef","right_gripper");
		eef_ = "right_gripper";
		ikFrame_ = "r_gripper_tool_frame";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef_, pipelinePlanner_}, {planGroup, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		grasp_generator->setAngleDelta(M_PI/2);
		pickTask.properties().exposeTo(grasp_generator->properties(), { "group","eef"});
	    grasp_generator->setPreGraspPose(pregrasp);
		grasp_generator->setGraspPose(grasp);
		grasp_generator->setProperty("object", object);
		grasp_generator->setMonitoredStage(current_state);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Isometry3d tr = Eigen::Isometry3d::Identity();
		pickTask.properties().exposeTo(grasp->properties(), { "group","eef","object"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group","object"});
		grasp->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		//tr.translation() = Eigen::Vector3d(0.0,0.0,0.00);
		grasp->setIKFrame(tr, ikFrame_);
		grasp->setMaxIKSolutions(10);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp),"pick");
		pickTask.properties().exposeTo(pick->properties(), { "group","eef","object" });
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group","object"});
		//pick->setProperty("eef", "left_gripper");
		//pick->setProperty("group","left_arm");
		//pick->setProperty("object", "obj_0");
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = ikFrame_;
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.10, 0.15);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.05, 0.10);
		current_state = pick.get();
		pickTask.add(std::move(pick));
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
	ROS_ERROR_STREAM("===============[BEWARE UPDATE OF THE WORLD INCOMING]==================");
	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
	shapes::Mesh* m;
	std::string meshURI;
	std::vector<std::string> meshTemp;

	moveit_msgs::CollisionObject collisionObj;

	moveit_msgs::CollisionObject throwBox_left;
	moveit_msgs::CollisionObject throwBox_right;

	geometry_msgs::PoseStamped colliObjPosetransformed;
	geometry_msgs::PoseStamped colliObjPoseUntransformed;

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

	std::vector<std::string> objIds = onto_.individuals.getOn(SUPPORT_SURFACE,"isUnder");

	// Add support surface to also add the table to the world
	objIds.push_back(SUPPORT_SURFACE);

	// Ask underworld about poses of these ids
	pr2_motion_tasks_msgs::GetPose srv;
	srv.request.ids = objIds;
	if (udwClient.call(srv))
	{
		for (int i=0; i < objIds.size(); i++)
    	{
			// Clear vector to be able to reuse it
			collisionObj.meshes.clear();
			collisionObj.mesh_poses.clear();

			// Fill in mesh URI (ask ontology or get it from the cache)
			//Verify if frame_id isn't empty
			// UWDS publish with frame_id as /map so transform to base_footprint
			if(!srv.response.poses[i].header.frame_id.empty())
			{
				// Ask the transform between map and basefootprint (as UWDS give object into the map frame)
				// Will wait for 1 seconds
				if(srv.response.poses[i].header.frame_id != "/base_footprint")
				{
					try
					{
					mainTransform_ = tfBuffer_.lookupTransform("base_footprint",srv.response.poses[i].header.frame_id.erase(0, 1), ros::Time(0),ros::Duration(5.0));
					}
					catch (tf2::TransformException &ex)
					{
						ROS_WARN("%s",ex.what());
						return 1;
					}
				}

				// Only ask if new object
				if (objMeshMap_.find(objIds[i]) != objMeshMap_.end())
				{
					// Mesh URI is already known so get it from the map
					m = shapes::createMeshFromResource(objMeshMap_.at(objIds[i]));
				}
				else
				{
					// Mesh URI is not known so ask ontologenius for it
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
					}
					else
					{
						ROS_ERROR_STREAM("Error while updating the world, no meshes were returned by Ontologenius...");
						return 2;
					}
				}

				shapes::constructMsgFromShape(m, mesh_msg);
				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

				// Add the mesh to the Collision object message
				collisionObj.meshes.push_back(mesh);

	     		// Set object id
				collisionObj.id = objIds[i];

				if(srv.response.poses[i].header.frame_id != "/base_footprint")
				{
					// Transform pose given by UWDS from map to basefootprint
					colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
					tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,mainTransform_);
					collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
				}
				else
				{
					collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
				}

				// Set frame_id to "base_footprint" as it has been transformed
				collisionObj.header.frame_id = "base_footprint";

        		collisionObj.operation = collisionObj.ADD;

				// Add synchronously the collision object to planning scene (wait for it to be added before continuing)
				planning_scene_interface_.applyCollisionObject(collisionObj);



	 		}
			else
			{
				ROS_ERROR_STREAM("Error while updating the world, underworld service returned nothing...");
				return 3;
			}
	 	}

		// Add the two box where to throw objects (not seen by perception)
		throwBox_left.id = "throw_box_left";
		throwBox_left.header.frame_id = "base_footprint";
		throwBox_left.operation = throwBox_left.ADD;
		m = shapes::createMeshFromResource("package://exp_director_task/mesh/throw_box.dae");
		shapes::constructMsgFromShape(m, mesh_msg);
		mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
		// Add the mesh to the Collision object message
		throwBox_left.meshes.push_back(mesh);
		geometry_msgs::PoseStamped throw_box_pose;
		throw_box_pose.header.frame_id= "base_footprint";
		throw_box_pose.pose.position.x = 0.0;
		throw_box_pose.pose.position.y = 0.7;
		throw_box_pose.pose.position.z = 0.0;
		throw_box_pose.pose.orientation.x = 0.0;
		throw_box_pose.pose.orientation.y = 0.0;
		throw_box_pose.pose.orientation.z = 0.0;
		throw_box_pose.pose.orientation.w = 1.0;
		throwBox_left.mesh_poses.push_back(throw_box_pose.pose);
		planning_scene_interface_.applyCollisionObject(throwBox_left);

		// Add the two box where to throw objects (not seen by perception)
		throwBox_right.id = "throw_box_right";
		throwBox_right.header.frame_id = "base_footprint";
		throwBox_right.operation = throwBox_right.ADD;
		m = shapes::createMeshFromResource("package://exp_director_task/mesh/throw_box.dae");
		shapes::constructMsgFromShape(m, mesh_msg);
		mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
		// Add the mesh to the Collision object message
		throwBox_right.meshes.push_back(mesh);
		throw_box_pose.header.frame_id= "base_footprint";
		throw_box_pose.pose.position.x = 0.0;
		throw_box_pose.pose.position.y = -0.7;
		throw_box_pose.pose.position.z = 0.0;
		throw_box_pose.pose.orientation.x = 0.0;
		throw_box_pose.pose.orientation.y = 0.0;
		throw_box_pose.pose.orientation.z = 0.0;
		throw_box_pose.pose.orientation.w = 1.0;
		throwBox_right.mesh_poses.push_back(throw_box_pose.pose);
		planning_scene_interface_.applyCollisionObject(throwBox_right);
	}
	else
	{
		ROS_ERROR("Failed to call service getPose");
		return 3;
	}
	return 0;
}

void taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat, int& progress)
{
	progress = (taskStat->stages[0].solved.size()/10.0)*100;
}


void motionPlanning::planFeedbackThread(std::string task_id, actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>* planServer)
{
	pr2_motion_tasks_msgs::planFeedback planFeedback;
	int progressValue=0;

	std::string statTopic = "/pr2_tasks_node/" + task_id + "/statistics";
	
	ros::Subscriber sub = nh_.subscribe<moveit_task_constructor_msgs::TaskStatistics>(statTopic, 10, boost::bind(taskStatisticCallback,_1,boost::ref(progressValue)));

	ros::Rate loop_rate(2);
	while (planServer->isActive())
	{
		if(planServer->isPreemptRequested())
		{
			lastPlannedTask_->preempt();
			return;
		}
		planFeedback.status = progressValue;
		planServer->publishFeedback(planFeedback);
		loop_rate.sleep();
	}
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
 * \fn void planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>* planServer, ros::ServiceClient& udwClient)
 * \brief Callback that is called when supervisor ask for a plan
 *
 * \param goal Goal sent by supervisor. Contains action to be planned (pick, place, move), planGroup to be used if moving, object if pick, box if place
 * \param planServer Action server handle to be able to send feeback or result to supervisor
 * \param udwClient Service handle to pass on to the update world function
 */
void motionPlanning::planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>* planServer, ros::ServiceClient& udwClient)
{
 	pr2_motion_tasks_msgs::planResult planResult;

	std::vector<geometry_msgs::PoseStamped> customPoses;
    geometry_msgs::PoseStamped customPose;

	int updateWorldResult = 0;

	std::string armGroup;
	std::string taskName;

	if((goal->action == "pick"))
	{
		//updateWorldResult = updateWorld(udwClient);
		if(updateWorldResult == 1)
		{
			planResult.error_code = -4;
			planServer->setAborted(planResult);
			return;
		}
		else if(updateWorldResult == 2)
		{
			planResult.error_code = -5;
			planServer->setAborted(planResult);
			return;
		}
		else if(updateWorldResult == 3)
		{
			planResult.error_code = -6;
			planServer->setAborted(planResult);
			return;
		}
	}

	if(goal->planGroup.empty())
	{
		std::vector<std::string> objIds;
		objIds.push_back(goal->objId);
		if(planning_scene_interface_.getObjectPoses(objIds).find(goal->objId)->second.position.y > 0)
		{
			armGroup = "left_arm";
		}
		else
		{
			armGroup = "right_arm";
		}
	}
	else
	{
		armGroup = goal->planGroup;
	}

	//====== PICK ======//
	if(goal->action == "pick")
	{
		customPose.header.frame_id = goal->objId;
		customPose.pose.position.x = 0.00;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.0;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 0.0;
		customPose.pose.orientation.w = 1.0;
		customPoses.push_back(customPose);


		customPose.header.frame_id = goal->objId;
		customPose.pose.position.x = 0.00;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.0;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 1.0;
		customPose.pose.orientation.w = 0.0;
		customPoses.push_back(customPose);

		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_shared<Task>(taskName);
		createPickTaskCustom(*lastPlannedTask_,armGroup,goal->objId, customPoses);
	}
	//====== PLACE ======//
	else if(goal->action == "place")
	{
		customPose.header.frame_id = goal->boxId;
		customPose.pose.position.x = 0.00;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.01;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 0.0;
		customPose.pose.orientation.w = 1.0;
		customPoses.push_back(customPose);


		customPose.header.frame_id = goal->boxId;
		customPose.pose.position.x = 0.00;
		customPose.pose.position.y = 0.0;
		customPose.pose.position.z = 0.01;
		customPose.pose.orientation.x = 0.0;
		customPose.pose.orientation.y = 0.0;
		customPose.pose.orientation.z = 1.0;
		customPose.pose.orientation.w = 0.0;
		customPoses.push_back(customPose);


		taskName = goal->action + "_" + goal->objId + "_in_" + goal->boxId;

		// Create Task
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		createPlaceTask(*lastPlannedTask_, armGroup, goal->objId, customPoses);
	}
	//====== MOVE ======//
	else if (goal->action == "move")
	{
		taskName = goal->action + "_" + goal->planGroup;
		// Create Task
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		if(goal->predefined_pose_id.empty())
		{
			createMoveTask(*lastPlannedTask_, armGroup,goal->pose);
		}
		else
		{
			createMovePredefinedTask(*lastPlannedTask_, armGroup,goal->predefined_pose_id);
		}
	}
	//====== DROP ======//
	else if (goal->action == "drop")
	{
		taskName = goal->action + "_" + goal->planGroup;
		// Create Task
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		createDropTask(*lastPlannedTask_, armGroup,goal->objId);
	}
	else
	{
		// TODO handle this case
		return;
	}

	// Create Thread to handle the feedback process 
	boost::thread feedbackThread(&motionPlanning::planFeedbackThread, this, taskName, planServer);

	try
	{
		if(lastPlannedTask_->plan(10) && !planServer->isPreemptRequested())
		{
			planResult.error_code = 1;
			planResult.cost = lastPlannedTask_->solutions().front()->cost();
			pr2_motion_tasks_msgs::planFeedback planFeedback;
			planFeedback.status = 100;
			planServer->publishFeedback(planFeedback);
			planServer->setSucceeded(planResult);
		}
		else
		{
			planResult.error_code = -1;
			
			if(planServer->isPreemptRequested())
			{
				planServer->setPreempted(planResult);
			}
			else
			{
				planServer->setAborted(planResult);
			}
		}
	}
	catch (const InitStageException& e)
	{
    // TODO Handle this state
		ROS_ERROR_STREAM(e);
	}
	feedbackThread.join();

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
 * \fn void executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>* executeServer)
 * \brief Callback that is called when supervisor ask to execute last planned task
 *
 * \param goal Goal sent by supervisor. Void
 * \param executeServer Action server handle to be able to send feeback or result to supervisor
 */
void motionPlanning::executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>* executeServer, ros::Publisher factsPublisher)
{
	pr2_motion_tasks_msgs::executeFeedback executeFeedback;
  	pr2_motion_tasks_msgs::executeResult executeResult;

	bool doneFlag = false;

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> executeTask("execute_task_solution", true);
	executeTask.waitForServer();

	// Verify that task had solutions
	if(lastPlannedTask_->solutions().size() > 0)
	{
    // TODO Check that this works
		ROS_INFO_STREAM("Executing solution trajectory of " << lastPlannedTask_->id());

		// Fill the solution message
		lastPlannedTask_->solutions().front()->fillMessage(execute_goal.solution);

		executeTask.sendGoal(execute_goal, boost::bind(&doneCb,_1,_2,boost::ref(doneFlag)), &activeCb, &feedbackCb);
		int dummyProgress = 0;
		ros::Rate loop_rate(1);
		while(!doneFlag)
		{
			executeFeedback.status = dummyProgress;
			dummyProgress++;
			executeServer->publishFeedback(executeFeedback);
			if(executeServer->isPreemptRequested())
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

			if(executeServer->isPreemptRequested())
			{
				executeServer->setPreempted(executeResult);
			}
			else
			{
				executeServer->setAborted(executeResult);
			}
			
		}
		else
		{
			executeResult.error_code = 1;
			executeServer->setSucceeded(executeResult);
			pr2_motion_tasks_msgs::StringStamped factStampedMsg;
			factStampedMsg.stamp = ros::Time::now();
			factStampedMsg.fact = lastPlannedTask_->id();
			factsPublisher.publish(factStampedMsg);
		}
	}
	else
	{
		executeResult.error_code = -3;
		executeServer->setAborted(executeResult);
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_tasks_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");
	ros::Rate r(10); // 10 hz+

	motionPlanning pr2Motion;

	// Service to get object pose from underworld
	ros::ServiceClient getPoseSrv = nh.serviceClient<pr2_motion_tasks_msgs::GetPose>("/tag_service/getPose");
	//ros::service::waitForService("/tag_service/getPose", -1);

  ros::Publisher facts_pub = nh.advertise<pr2_motion_tasks_msgs::StringStamped>("pr2_facts", 1000);

	// Action servers for supervisor
  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction> planServer(nh, "plan", boost::bind(&motionPlanning::planCallback, &pr2Motion, _1, &planServer, getPoseSrv), false);
	planServer.start();

  actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction> executeServer(nh, "execute", boost::bind(&motionPlanning::executeCallback, &pr2Motion, _1, &executeServer, facts_pub), false);
	executeServer.start();

	//pr2Motion.updateWorld(getPoseSrv);


	ROS_ERROR("STARTED ACTION SERVS");

	while(ros::ok());

	return 0;
}

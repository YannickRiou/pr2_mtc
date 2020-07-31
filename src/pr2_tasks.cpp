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

#include <pr2_mtc/pr2_tasks.h>


/**
 * \fn void createPlaceTask(Task &placeTask, const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian, const moveit::core::RobotModelPtr& robotModel, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose)
 * \brief Function to create a place task with specific pose and object
 *
 * \param placeTask Task to fill
 * \param pipeline planner to be used to connect stages
 * \param cartesian planner to be used for approach and retreat stages
 * \param robotModel Model of the robot (from urdf)
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to be placed (it needs to be already attached to the eef of the associated planGroup)
 * \param placePose Pose where to place the object
 */
void createPlaceTask(Task &placeTask, const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian, const moveit::core::RobotModelPtr& robotModel, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose)
{

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	placeTask.setRobotModel(robotModel);

	// Property and variable definitions
	std::string eef;
	std::string ungrasp;
	std::string ikFrame;

	placeTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","left_gripper");
	    eef = "left_gripper";
		ikFrame = "l_gripper_tool_frame";
		ungrasp = "left_open";
	}
	else if(planGroup == "right_arm")
	{
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		ikFrame = "r_gripper_tool_frame";
		ungrasp = "right_open";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	placeTask.properties().exposeTo(initial->properties(), { "eef", "group" });
	placeTask.add(std::move(initial));

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		placeTask.add(std::move(connect));
	}


	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("place pose");
		stage->setPose(placePose);
		stage->setObject(object);
		placeTask.properties().exposeTo(stage->properties(), { "eef", "group"});

		geometry_msgs::PoseStamped ik;
		ik.header.frame_id= ikFrame;
		ik.pose.position.x=  0;
		ik.pose.position.y=  0;
		ik.pose.position.z=  0;
		stage->setProperty("ik_frame",ik);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage));
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(ikFrame);
		wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		placeTask.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner);
		stage->setGroup(eef);
		stage->setGoal(ungrasp);
		placeTask.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, ikFrame);
		placeTask.add(std::move(stage));
	}
}

void createMoveTask(Task &moveTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup, const geometry_msgs::PoseStamped moveToPose)
{
	moveTask.setRobotModel(robotModel);
	std::string ikFrame;
		std::string eef;
	if(planGroup == "left_arm")
	{
		ikFrame = "l_gripper_tool_frame";
		eef = "left_gripper";
	}
	else if(planGroup == "right_arm")
	{
		ikFrame = "r_gripper_tool_frame";
		eef = "right_gripper";
	}

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	moveTask.add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline},{eef, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		moveTask.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef);
		stage->setPose(moveToPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(ikFrame);
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		moveTask.add(std::move(wrapper));
	}
}

void createPickTaskCustom(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose)
{
	pickTask.setRobotModel(robotModel);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// Property and variable definitions
	std::string eef;
	std::string ikFrame;
	std::string pregrasp;
	std::string postgrasp;

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","left_gripper");
	    eef = "left_gripper";
		pregrasp = "left_open";
		postgrasp = "left_close";
		ikFrame = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		pregrasp = "right_open";
		postgrasp = "right_close";
		ikFrame = "r_gripper_tool_frame";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", gripper_planner);
		stage->setGroup(eef);
		stage->setGoal(pregrasp);
		current_state = stage.get();
		pickTask.add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline},{eef, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->setTimeout(10.0);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask.add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object");

		pickTask.properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.08);
			stage->setIKFrame(ikFrame);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = object;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::GeneratePose>("go to grasp pose");
			stage->setPose(graspPose);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame);
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask.getRobotModel()->getJointModelGroup(eef)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner);
			stage->setGroup(eef);
			stage->setGoal(postgrasp);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Attach Object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame);
			current_state = stage.get();
			grasp->insert(std::move(stage));
		}

		// ---------------------- Allow collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, "boite", true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesian);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.15, 0.20);
			stage->setIKFrame(ikFrame);
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
void createPickTask(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel, const std::string planGroup,const std::string object)
{
	pickTask.setRobotModel(robotModel);

	// Property and variable definitions
	std::string eef;
	std::string ikFrame;
	std::string pregrasp;
	std::string grasp;

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "left_open";
		grasp = "left_close";
		pickTask.setProperty("eef","left_gripper");
	  eef = "left_gripper";
		ikFrame = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "right_open";
		grasp = "right_close";
		pickTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		ikFrame = "r_gripper_tool_frame";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));

	{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (arm,support)");
			stage->allowCollisions(
			    "tableLaas",
			    pickTask.getRobotModel()->getJointModelGroup(planGroup)->getLinkModelNamesWithCollisionGeometry(), false);
			pickTask.add(std::move(stage));
	}


	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {planGroup, pipeline}};
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
		grasp->setIKFrame(tr, ikFrame);
		grasp->setMaxIKSolutions(10);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp),"pick");
		pickTask.properties().exposeTo(pick->properties(), { "group","eef","object" });
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group","object"});
		//pick->setProperty("eef", "left_gripper");
		//pick->setProperty("group","left_arm");
		//pick->setProperty("object", "obj_0");
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = ikFrame;
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

int execute(Task &t)
{

	if(t.solutions().size() > 0)
	{
		actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution", true);
		ac.waitForServer();
		ROS_INFO("Executing solution trajectory");
		moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
		t.solutions().front()->fillMessage(execute_goal.solution);
		ac.sendGoal(execute_goal);
		ac.waitForResult();
		moveit_msgs::MoveItErrorCodes execute_result = ac.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_ERROR_STREAM("Task execution failed and returned: " << ac.getState().toString());
			return -1;
		}
		return 0;
	}

}

//Open the gripper
bool gripper_open(GripperClient* gripper)
{
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
  bool success;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)

  gripper->sendGoal(open);
  gripper->waitForResult();

  if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //ROS_INFO("gripper opened!");
    success = true;
  }
  else
  {
    //ROS_ERROR("gripper failed to open.");
    success = false;
  }

  return success;
}

//Close the gripper
bool gripper_close(GripperClient* gripper, float effort)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;

  bool success;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = effort;  // Close gently

  gripper->sendGoal(squeeze);
  gripper->waitForResult();
  if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Right gripper closed!");
     success = true;
  }
  else
  {
    ROS_ERROR("Right gripper failed to close.");
    success = false;
  }

  return success;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_task");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");
	ros::Rate r(10); // 10 hz

	// Parameter for test only
	std::string left_obj;
	std::string right_obj;
	std::string choosedPlanner;

	nh.getParam("left", left_obj);
  nh.getParam("right", right_obj);
	nh.getParam("planner", choosedPlanner);

	// Create a perception handler and launch it
	pr2perception perception(nh);
	perception.startPerception();
	ros::Duration(6).sleep();

	// Robot model shared by all tasks
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

	// planner used for approach and retreat
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId(choosedPlanner);

	geometry_msgs::PoseStamped graspPose_left;
	graspPose_left.header.frame_id = left_obj;
	graspPose_left.pose.position.x = 0.00;
	graspPose_left.pose.position.y = 0.0;
	graspPose_left.pose.position.z = 0.03;
	graspPose_left.pose.orientation.x = 0.500;
	graspPose_left.pose.orientation.y = 0.500;
	graspPose_left.pose.orientation.z = -0.500;
	graspPose_left.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped graspPose_left_2;
	graspPose_left_2.header.frame_id = "obj_230";
	graspPose_left_2.pose.position.x = 0.00;
	graspPose_left_2.pose.position.y = 0.0;
	graspPose_left_2.pose.position.z = 0.03;
	graspPose_left_2.pose.orientation.x = 0.500;
	graspPose_left_2.pose.orientation.y = 0.500;
	graspPose_left_2.pose.orientation.z = -0.500;
	graspPose_left_2.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped movePose_left;
	movePose_left.header.frame_id = left_obj;
	movePose_left.pose.position.x = 0.00;
	movePose_left.pose.position.y = 0.0;
	movePose_left.pose.position.z = 0.10;
	movePose_left.pose.orientation.x = 0.500;
	movePose_left.pose.orientation.y = 0.500;
	movePose_left.pose.orientation.z = -0.500;
	movePose_left.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped movePose_left_2;
	movePose_left_2.header.frame_id = "obj_230";
	movePose_left_2.pose.position.x = 0.0;
	movePose_left_2.pose.position.y = 0.0;
	movePose_left_2.pose.position.z = 0.10;
	movePose_left_2.pose.orientation.x = 0.500;
	movePose_left_2.pose.orientation.y = 0.500;
	movePose_left_2.pose.orientation.z = -0.500;
	movePose_left_2.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped placePoseBox_left;
	placePoseBox_left.header.frame_id = "base_footprint";
	placePoseBox_left.pose.position.x = 0.3;
	placePoseBox_left.pose.position.y = 0.9;
	placePoseBox_left.pose.position.z = 1.1;
	placePoseBox_left.pose.orientation.x = 0.500;
	placePoseBox_left.pose.orientation.y = 0.500;
	placePoseBox_left.pose.orientation.z = 0.500;
	placePoseBox_left.pose.orientation.w = 0.500;

	// Create task objects
	Task moveLeft("moveLeft");
	Task moveLeft_next("moveLeft_next");

	Task pick_left("pick_left");
	Task pick_left_next("pick_left_next");

	Task place_left("place_left");
	Task place_left_next("place_left_next");

	try
	{
		createMoveTask(moveLeft,pipeline, cartesian,kinematic_model,"left_arm", movePose_left);
		createMoveTask(moveLeft_next,pipeline, cartesian,kinematic_model,"left_arm", movePose_left_2);

		createPickTaskCustom(pick_left,pipeline, cartesian,kinematic_model,"left_arm",left_obj, graspPose_left);
		createPickTaskCustom(pick_left_next,pipeline, cartesian,kinematic_model,"left_arm","obj_230", graspPose_left_2);

		createPlaceTask(place_left,pipeline,cartesian,kinematic_model,"left_arm",left_obj,placePoseBox_left);
		createPlaceTask(place_left_next,pipeline,cartesian,kinematic_model,"left_arm","obj_230",placePoseBox_left);

		// Stop perception before moving objects (avoid to create new collision objects)
		perception.stopPerception();

		// Plan and execute approach movement
		moveLeft.plan(2);
		if(execute(moveLeft) == 0)
		{
			pick_left.plan(2);
			if(execute(pick_left) == 0)
			{
				place_left.plan(2);
				if(execute(place_left) == 0)
				{
					perception.startPerception();
					ros::Duration(5).sleep();
					perception.stopPerception();
					moveLeft_next.plan(2);
					execute(moveLeft_next);

					pick_left_next.plan(2);
					if(execute(pick_left_next) == 0)
					{
						place_left_next.plan(2);
						execute(place_left_next);
					}
				}
			}
		}

	}
	catch (const InitStageException& e)
	{
		ROS_ERROR_STREAM(e);
	}

	while(ros::ok());

	return 0;
}

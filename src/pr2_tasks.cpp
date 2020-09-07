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

// TODO Add missing TF2 transform publisher
motionPlanning::motionPlanning(ros::NodeHandle nh)
  : nh_(nh),
	robot_model_loader_("robot_description")
{
	kinematic_model_ = robot_model_loader_.getModel();

	cartesianPlanner_ = std::make_shared<solvers::CartesianPath>();
	cartesianPlanner_->setProperty("jump_threshold", 0.0);

	pipelinePlanner_ = std::make_shared<solvers::PipelinePlanner>();
	pipelinePlanner_->setPlannerId("RRTConnect");

	gripper_planner_ = std::make_shared<solvers::JointInterpolationPlanner>();
}


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
void motionPlanning::createPlaceTask(Task &placeTask, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose)
{
	placeTask.setRobotModel(kinematic_model_);

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
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_}};
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
		auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
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

void motionPlanning::createMoveTask(Task &moveTask, const std::string planGroup, const geometry_msgs::PoseStamped moveToPose)
{
	moveTask.setRobotModel(kinematic_model_);
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
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef, gripper_planner_}};
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

void motionPlanning::createPickTaskCustom(Task &pickTask, const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose)
{
	pickTask.setRobotModel(kinematic_model_);

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
		auto stage = std::make_unique<stages::MoveTo>("open hand", gripper_planner_);
		stage->setGroup(eef);
		stage->setGoal(pregrasp);
		current_state = stage.get();
		pickTask.add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef, gripper_planner_}};
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
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
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
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); 
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
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
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
		/*{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, "boite", true);
			grasp->insert(std::move(stage));
		}*/

			// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
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
			auto stage = std::make_unique<stages::MoveRelative>("retreat object", cartesianPlanner_);
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
void motionPlanning::createPickTask(Task &pickTask, const std::string planGroup,const std::string object)
{
	pickTask.setRobotModel(kinematic_model_);

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
		stages::Connect::GroupPlannerVector planners = {{eef, pipelinePlanner_}, {planGroup, gripper_planner_}};
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
		return 1;
	}

}


// Function to ask ontologenius about object id/meshes that are on the table 
// then ask underworld their positions, and add them to planning scene

void motionPlanning::updateWorld(ros::ServiceClient& udwClient, OntologyManipulator* ontoHandle)
{

	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
	shapes::Mesh* m;

	// TODO add cache with a map id/mesh_uri so that I don't need to ask ontologenius twice

	// Define a collision object that will be added to planning scene
	moveit_msgs::CollisionObject collisionObj;

	// TODO CHANGE TABLE WITH PROPER SUPPORT SURFACE NAME (CHECK WITH ONTOLOGY)
	std::vector<std::string> objIds = ontoHandle->individuals.getOn(SUPPORT_SURFACE,"isUnder");
	
	pr2_mtc::getPose srv;
	srv.request.ids = objIds;
	if (udwClient.call(srv))
	{
		for (int i=0; i < objIds.size(); i++)
    	{
			//Verify if frame_id isn't empty
			// Assume that frame_id is base_footprint 
			// UWDS publish with frame_id as /map so transform to base_footprint 
			// TODO add transform from /map to /base_footprint
			if(!srv.response.poses[i].header.frame_id.empty())
			{
				// Ask ontology for mesh ressource URI
				m = shapes::createMeshFromResource(ontoHandle->individuals.getOn(objIds[i],"hasMesh")[0]);
				shapes::constructMsgFromShape(m, mesh_msg);
				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
				// Add the mesh to the Collision object message
				collisionObj.meshes.push_back(mesh);

				// Add poses given by UWDS	
				collisionObj.header.frame_id = srv.response.poses[i].header.frame_id;
				collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
				collisionObj.operation = collisionObj.ADD;

				planning_scene_interface_.applyCollisionObject(collisionObj);
			}
		}
		
	}
	else
	{
	ROS_ERROR("Failed to call service getPose");
	}

}

/*void feedback()
{

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
   
    loop_rate.sleep();
  }
}*/

void solutionCallback(const moveit_task_constructor_msgs::SolutionConstPtr& solution, int& cost)
{
  cost = solution->sub_solution[0].info.cost;
}


// TODO : Create Class to avoid huge number of parameter (let pipelineplanner, cartesianplanner, etc. be attribute of the class)
void motionPlanning::pickObjCallback(const pr2_mtc::pickGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::pickAction>* pickServer, ros::ServiceClient& udwClient, OntologyManipulator* ontoHandle)
{
	// First update the world
	updateWorld(udwClient,ontoHandle);

	pr2_mtc::pickFeedback pickFeedback;
  	pr2_mtc::pickResult pickResult;

    int solutionCost;

	std::string taskName = "pick_" + goal->objId;
	std::string armGroup;

	ros::Subscriber solutionSub;

	Task pick(taskName);

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

	// TODO : Checks that the orientation of added object is correct
	geometry_msgs::PoseStamped pickPose;
	pickPose.header.frame_id = goal->objId;
	pickPose.pose.position.x = 0.00;
	pickPose.pose.position.y = 0.0;
	pickPose.pose.position.z = 0.03;      // TBC
	pickPose.pose.orientation.x = 0.500;  // TBC
	pickPose.pose.orientation.y = 0.500;  // TBC
	pickPose.pose.orientation.z = -0.500; // TBC
	pickPose.pose.orientation.w = 0.500;  // TBC


	createPickTaskCustom(pick,armGroup,goal->objId, pickPose);
	
	// TODO give info to supervisor about cost, trajectory/

	if(goal->planOnly)	
	{	

		// TODO : spawn another thread to plan and send back to supervisor cost and trajectory
		//boost::thread feedbackThread(feedback);

		if(pick.plan(5))
		{
			pick.publishAllSolutions(false);
		
			solutionSub = nh_.subscribe<moveit_task_constructor_msgs::Solution>("/pr2_task_node/" + taskName + "/solution", 1000, boost::bind(solutionCallback,_1, solutionCost));
			pickResult.cost = solutionCost;	

			moveit::planning_interface::MoveItErrorCode result(1);
			pickResult.error_code = result;

			pickServer->setSucceeded(pickResult);
		}
		else
		{
			moveit::planning_interface::MoveItErrorCode result(-1);
			pickResult.error_code = result;
			pickServer->setAborted(pickResult);
		}
	}	
	else
	{
		if(execute(pick))
		{
			moveit::planning_interface::MoveItErrorCode result(1);
			pickResult.error_code = result;
			pickServer->setSucceeded(pickResult);
		}
		else
		{
			moveit::planning_interface::MoveItErrorCode result(-1);
			pickResult.error_code = result;
			pickServer->setAborted(pickResult);
		}
	}
}


void motionPlanning::placeObjCallback(const pr2_mtc::placeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::placeAction>* placeServer, ros::ServiceClient& udwClient, OntologyManipulator* ontoHandle)
{
	// First update the world
	updateWorld(udwClient,ontoHandle);

	std::string taskName = "place_into_" + goal->boxId;
	Task place(taskName);

	if(goal->planOnly)	
	{

	}	
	else
	{

	}
}

void motionPlanning::moveCallback(const pr2_mtc::moveGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::moveAction>* moveServer, ros::ServiceClient& udwClient, OntologyManipulator* ontoHandle)
{
	// First update the world
	updateWorld(udwClient,ontoHandle);

	std::string taskName = "move_" + goal->planGroup;
	Task move(taskName);
	
	if(goal->planOnly)	
	{

	}	
	else
	{

	}
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_task_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();
 
	ros::NodeHandle nh("~");
	ros::Rate r(10); // 10 hz

	motionPlanning pr2Motion(nh);

	// Ontologenius handle
	OntologyManipulator* onto_;
	OntologyManipulator onto(&nh);
	onto_ = &onto;
	onto.close();

 
	// Service to get object pose from underworld
	ros::ServiceClient getPoseSrv = nh.serviceClient<pr2_mtc::getPose>("getPose");

	// Action servers for supervisor 
  	actionlib::SimpleActionServer<pr2_mtc::pickAction> pickServer(nh, "pick", boost::bind(&motionPlanning::pickObjCallback, &pr2Motion, _1, &pickServer, getPoseSrv, onto_), false);
	actionlib::SimpleActionServer<pr2_mtc::placeAction> placeServer(nh, "place", boost::bind(&motionPlanning::placeObjCallback, &pr2Motion, _1, &placeServer,getPoseSrv, onto_), false);
  	actionlib::SimpleActionServer<pr2_mtc::moveAction> moveServer(nh, "move", boost::bind(&motionPlanning::moveCallback, &pr2Motion, _1, &moveServer,getPoseSrv, onto_), false);

	

	try
	{

	}
	catch (const InitStageException& e)
	{
		ROS_ERROR_STREAM(e);
	}

	while(ros::ok());

	return 0;
}

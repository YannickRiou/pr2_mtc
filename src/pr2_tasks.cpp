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

motionPlanning::motionPlanning()
  : onto_(&nh_),
	robot_model_loader_("robot_description"),
	transformListenner_(tfBuffer_)
{
	onto_.close();
	onto_.verbose(true);
	kinematic_model_ = robot_model_loader_.getModel();

	cartesianPlanner_ = std::make_shared<solvers::CartesianPath>();
	cartesianPlanner_->setProperty("jump_threshold", 0.0);

	pipelinePlanner_ = std::make_shared<solvers::PipelinePlanner>();
	pipelinePlanner_->setPlannerId("RRTConnect");
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.01);

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

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.001);


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

void motionPlanning::createMoveTask(Task &moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
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
		postgrasp = "right_close";
		ikFrame = "r_gripper_tool_frame";
	}


	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.001);

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

// Function to ask ontologenius about object id/meshes that are on the table 
// then ask underworld their positions, and add them to planning scene

void motionPlanning::updateWorld(ros::ServiceClient& udwClient)
{

	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
	shapes::Mesh* m;
	std::string meshURI;
	std::vector<std::string> meshTemp;

	moveit_msgs::CollisionObject collisionObj;


	geometry_msgs::PoseStamped colliObjPosetransformed;
	geometry_msgs::PoseStamped colliObjPoseUntransformed;


	// Ask the transform between map and basefootprint (as UWDS give object into the map frame)
	try
	{
	mainTransform_ = tfBuffer_.lookupTransform("base_footprint","map",ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
	}

	// Define a collision object that will be added to planning scene
	

	std::vector<std::string> objIds = onto_.individuals.getOn(SUPPORT_SURFACE,"isUnder");
	
	objIds.push_back(SUPPORT_SURFACE);

	pr2_mtc::getPose srv;
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
				// Only ask if new object
				if (objMeshMap_.find(objIds[i]) != objMeshMap_.end())
				{
					ROS_INFO("ALREADY KNOWN OBJECT, GET IT FROM MAP");
					// Mesh URI is already known so get it from the map
					m = shapes::createMeshFromResource(objMeshMap_.at(objIds[i]));
				} 
				else 
				{
					ROS_INFO("UNKNOWN OBJECT CALL ONTOLOGENIUS");
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
					}
				}

				shapes::constructMsgFromShape(m, mesh_msg);
				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

				// Add the mesh to the Collision object message
				collisionObj.meshes.push_back(mesh);

	     		// Set object id 
				collisionObj.id = objIds[i];

				// Transform pose given by UWDS from map to basefootprint	
				colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
				tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,mainTransform_);

				// Set frame_id to "base_footprint" as it has been transformed
				collisionObj.header.frame_id = "base_footprint";
				collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
				collisionObj.operation = collisionObj.ADD;

				// Add synchronously the collision object to planning scene (wait for it to be added before continuing)
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
	for(int i=0; i < solution->sub_solution.size(); i++)
	{
		 ROS_ERROR_STREAM("COST RECEIVED[" << i << "] : " << solution->sub_solution[i].info.cost);
	}
  cost = solution->sub_solution[0].info.cost;
}

void motionPlanning::planCallback(const pr2_mtc::planGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::planAction>* planServer, ros::ServiceClient& udwClient)
{
	// First update the world
	updateWorld(udwClient);

	pr2_mtc::planFeedback planFeedback;
  	pr2_mtc::planResult planResult;

    std::string armGroup;

  

	//====== PICK ======//
	if(goal->action == "pick")
	{ 
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

		// TODO : Checks that the orientation of added object is correct
		geometry_msgs::PoseStamped pickPose;
		pickPose.header.frame_id = goal->objId;
		pickPose.pose.position.x = -0.03;
		pickPose.pose.position.y = 0.0;
		pickPose.pose.position.z = 0.0;      // TBC
		pickPose.pose.orientation.x = 0.0;   // TBC
		pickPose.pose.orientation.y = 0.0;   // TBC
		pickPose.pose.orientation.z = 0.0;   // TBC
		pickPose.pose.orientation.w = 1.0;   // TBC

		std::string taskName = goal->action + "_" + goal->objId;

		// Create Task 
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		createPickTaskCustom(*lastPlannedTask_,armGroup,goal->objId, pickPose);
	}
	//====== PLACE ======//
	else if(goal->action == "place")
	{
		// TODO : Checks that the orientation of box is correct
		geometry_msgs::PoseStamped placePose;
		placePose.header.frame_id = goal->boxId;
		placePose.pose.position.x = 0.0;
		placePose.pose.position.y = -0.03;
		placePose.pose.position.z = 0.0;
		placePose.pose.orientation.x = 0.0;  	// TBC
		placePose.pose.orientation.y = 0.0; 	// TBC
		placePose.pose.orientation.z = 0.707; 	// TBC
		placePose.pose.orientation.w = 0.707;   // TBC

		std::string taskName = goal->action + "_in_" + goal->boxId;

		// Create Task 
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		createPlaceTask(*lastPlannedTask_, goal->planGroup, goal->objId, placePose);
	}
	//====== MOVE ======//
	else if (goal->action == "move")
	{
		std::string taskName = goal->action + "_" + goal->planGroup;
		// Create Task 
		lastPlannedTask_ = std::make_shared<Task>(taskName);

		createMoveTask(*lastPlannedTask_, goal->planGroup,goal->pose);
	}

	try
	{
		if(lastPlannedTask_->plan(2))
		{
			moveit::planning_interface::MoveItErrorCode result(1);
			planResult.error_code = result;
			planResult.cost = lastPlannedTask_->solutions().front()->cost();	
			planServer->setSucceeded(planResult);
		}
		else
		{
			moveit::planning_interface::MoveItErrorCode result(-1);
			planResult.error_code = result;
			planServer->setAborted(planResult);
		}
	}
	catch (const InitStageException& e)
	{
		ROS_ERROR_STREAM(e);
	}

}

void motionPlanning::executeCallback(const pr2_mtc::executeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_mtc::executeAction>* executeServer)
{
	pr2_mtc::executeFeedback executeFeedback;
  	pr2_mtc::executeResult executeResult;

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> executeTask("execute_task_solution", true);
	executeTask.waitForServer();

	// Verify that task had solutions
	if(lastPlannedTask_->solutions().size() > 0)
	{		
		ROS_INFO("Executing solution trajectory");

		// Fill the solution message
		lastPlannedTask_->solutions().front()->fillMessage(execute_goal.solution);

		executeTask.sendGoal(execute_goal);

		// TODO maybe add a timeout to avoid blocking
		executeTask.waitForResult();

		moveit_msgs::MoveItErrorCodes execute_result = executeTask.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) 
		{
			ROS_ERROR_STREAM("Task execution failed and returned: " << executeTask.getState().toString());
			moveit::planning_interface::MoveItErrorCode result(99999);
			executeResult.error_code = result;
			executeServer->setAborted(executeResult);
		}
		moveit::planning_interface::MoveItErrorCode result(1);
		executeResult.error_code = result;
		executeServer->setAborted(executeResult);
	}
	else
	{
		moveit::planning_interface::MoveItErrorCode result(-2);
		executeResult.error_code = result;
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
	ros::ServiceClient getPoseSrv = nh.serviceClient<pr2_mtc::getPose>("/underworldServer/getPose");
	ros::service::waitForService("/underworldServer/getPose", -1);

	// Action servers for supervisor 
  	actionlib::SimpleActionServer<pr2_mtc::planAction> planServer(nh, "plan", boost::bind(&motionPlanning::planCallback, &pr2Motion, _1, &planServer, getPoseSrv), false);
	planServer.start();
	
  	actionlib::SimpleActionServer<pr2_mtc::executeAction> executeServer(nh, "execute", boost::bind(&motionPlanning::executeCallback, &pr2Motion, _1, &executeServer), false);
	executeServer.start();

	ROS_ERROR("STARTED ACTION SERVS");

	while(ros::ok());

	return 0;
}

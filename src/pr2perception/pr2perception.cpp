/**
 * \file pr2_perception.cpp
 * \brief Handling of PR2 perception.
 * \author Yannick R.
 * \version 0.1
 * \date 22/08/20
 *
 * Scout "visualization_markers" topics and publish object as collision objects in moveit planning scene
 *
 */

#include <pr2_mtc/pr2perception.h>

pr2perception::pr2perception(ros::NodeHandle nh) : nh_(nh), tableListenner_(tfBuffer_)
{
	try
	{
	tableTransform_ = tfBuffer_.lookupTransform("base_footprint","head_mount_kinect2_rgb_optical_frame",ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
	}

	// Subscribe to topic giving the marker (RoboSherlock) to show the object as boxes in Rviz
	marker_sub_ = nh_.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(&pr2perception::markerCallback,this,_1,boost::ref(tableTransform_)));
}

// Class destructor
pr2perception::~pr2perception()
{




}


void pr2perception::markerCallback(const visualization_msgs::MarkerConstPtr& marker, geometry_msgs::TransformStamped& transform)
{
  // Add table as a fixed collision object.
  moveit_msgs::CollisionObject collisionObj;
  geometry_msgs::Pose collisionObj_pose;

  geometry_msgs::PoseStamped colliObjPosetransformed;
  geometry_msgs::PoseStamped colliObjPoseUntransformed;

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::Mesh* m;

  collisionObj.header.frame_id = "base_footprint";
  shape_msgs::SolidPrimitive collisionObj_primitive;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;


	// From ar_track_alvar
	if(marker->ns == "main_shapes")
	{
		if(marker->id == 54)
		{
			collisionObj.id = "tableLaas";

			std::string mesh_uri("package://pr2_mtc/meshes/ikea_table.dae");
			m = shapes::createMeshFromResource(mesh_uri);
			shapes::constructMsgFromShape(m, mesh_msg);
			mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
			// Add the mesh to the Collision object message
			collisionObj.meshes.push_back(mesh);

			// Define a pose for the object (specified relative to frame_id)
			colliObjPoseUntransformed.pose = marker->pose;

			tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);

			collisionObj_pose = colliObjPosetransformed.pose;
			collisionObj_pose.position.z = collisionObj_pose.position.z - 0.75;

			collisionObj.mesh_poses.push_back(collisionObj_pose);

			collisionObj.operation = collisionObj.ADD;

			if(perceptionAskedFlag_)
			{
				collision_objects_vector_.push_back(collisionObj);
				//publish tf
				static_transformStamped.header.stamp = colliObjPosetransformed.header.stamp;
				static_transformStamped.header.frame_id = "base_footprint";
				static_transformStamped.child_frame_id = collisionObj.id;
				static_transformStamped.transform.translation.x = collisionObj_pose.position.x;
				static_transformStamped.transform.translation.y = collisionObj_pose.position.y;
				static_transformStamped.transform.translation.z = collisionObj_pose.position.z;
				static_transformStamped.transform.rotation.x = collisionObj_pose.orientation.x;
				static_transformStamped.transform.rotation.y = collisionObj_pose.orientation.y;
				static_transformStamped.transform.rotation.z = collisionObj_pose.orientation.z;
				static_transformStamped.transform.rotation.w = collisionObj_pose.orientation.w;
				static_broadcaster.sendTransform(static_transformStamped);
			}


		}
		else
		{

			// Define a pose for the object (specified relative to frame_id)
			colliObjPoseUntransformed.pose = marker->pose;

			tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);

			collisionObj_pose = colliObjPosetransformed.pose;

			if(marker->id == 33)
			{
				collisionObj.id = "boite";

				std::string mesh_uri("package://pr2_mtc/meshes/director_box.dae");
				m = shapes::createMeshFromResource(mesh_uri);
				shapes::constructMsgFromShape(m, mesh_msg);
				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
				// Add the mesh to the Collision object message
				collisionObj.meshes.push_back(mesh);

				collisionObj_pose.orientation.z = 0.707;
				collisionObj_pose.orientation.w = 0.707;

				collisionObj.mesh_poses.push_back(collisionObj_pose);
			}
			else if(marker->id == 230)
			{
				collisionObj.id = "obj_230";
				collisionObj_primitive.type = collisionObj_primitive.BOX;
		   	collisionObj_primitive.dimensions.resize(3);
		   	collisionObj_primitive.dimensions[0] = 0.055;
				collisionObj_primitive.dimensions[1] = 0.055;
				collisionObj_primitive.dimensions[2] = 0.055*2;

				collisionObj_pose.orientation.z = 0.707;
				collisionObj_pose.orientation.w = 0.707;

				collisionObj.primitives.push_back(collisionObj_primitive);
				collisionObj.primitive_poses.push_back(collisionObj_pose);
			}
			else if(marker->id == 206)
			{
				collisionObj.id = "obj_206";
				collisionObj_primitive.type = collisionObj_primitive.BOX;
		   	 	collisionObj_primitive.dimensions.resize(3);
		   		collisionObj_primitive.dimensions[0] = 0.055;
				collisionObj_primitive.dimensions[1] = 0.055*2;
				collisionObj_primitive.dimensions[2] = 0.055;

				collisionObj.primitives.push_back(collisionObj_primitive);
				collisionObj.primitive_poses.push_back(collisionObj_pose);
			}


			collisionObj.operation = collisionObj.ADD;

			if(perceptionAskedFlag_)
			{
				collision_objects_vector_.push_back(collisionObj);
				//publish tf
				static_transformStamped.header.stamp = colliObjPosetransformed.header.stamp;
				static_transformStamped.header.frame_id = "base_footprint";
				static_transformStamped.child_frame_id = collisionObj.id;
				static_transformStamped.transform.translation.x = collisionObj_pose.position.x;
				static_transformStamped.transform.translation.y = collisionObj_pose.position.y;
				static_transformStamped.transform.translation.z = collisionObj_pose.position.z;
				static_transformStamped.transform.rotation.x = collisionObj_pose.orientation.x;
				static_transformStamped.transform.rotation.y = collisionObj_pose.orientation.y;
				static_transformStamped.transform.rotation.z = collisionObj_pose.orientation.z;
				static_transformStamped.transform.rotation.w = collisionObj_pose.orientation.w;
				static_broadcaster.sendTransform(static_transformStamped);
			}
		}
	}
	// OBJ is a primitive shape (box or cylinder)
	else if(marker->text != "")
	{
		collisionObj.id = marker->text;
		// Check the type of the marker. We search for CUBE and CYLINDER only
		if(marker->type == visualization_msgs::Marker::CUBE)
		{
			collisionObj_primitive.type = collisionObj_primitive.BOX;
			collisionObj_primitive.dimensions.resize(3);
			collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker->scale.z;
		}
		else if(marker->type == visualization_msgs::Marker::CYLINDER)
		{
			collisionObj_primitive.type = collisionObj_primitive.CYLINDER;
			collisionObj_primitive.dimensions.resize(2);
			collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] =marker->scale.y /2.0;
			collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = marker->scale.z;

			/*
			std::cout << "Object name is :" << collisionObj.id << std::endl;
			std::cout << "Object Size is :" << std::endl;
			std::cout << "Radius :" << marker->scale.x/2.0 << std::endl;
			std::cout << "Height :" <<  marker->scale.z << std::endl;
			*/
		}
		else
		{
			return;
		}

		// Define a pose for the object (specified relative to frame_id)

		collisionObj_pose.position.x = marker->pose.position.x;
		collisionObj_pose.position.y = marker->pose.position.y;
		collisionObj_pose.position.z = marker->pose.position.z+0.02;

		collisionObj_pose.orientation.x = marker->pose.orientation.x;
		collisionObj_pose.orientation.y = marker->pose.orientation.y;
		collisionObj_pose.orientation.z = marker->pose.orientation.z;
		collisionObj_pose.orientation.w = marker->pose.orientation.w;

		collisionObj.primitives.push_back(collisionObj_primitive);
		collisionObj.primitive_poses.push_back(collisionObj_pose);

		collisionObj.operation = collisionObj.ADD;

		if(perceptionAskedFlag_)
		{
			collision_objects_vector_.push_back(collisionObj);
			//publish tf
			static_transformStamped.header.stamp = colliObjPosetransformed.header.stamp;
			static_transformStamped.header.frame_id = "base_footprint";
			static_transformStamped.child_frame_id = collisionObj.id;
			static_transformStamped.transform.translation.x = collisionObj_pose.position.x;
			static_transformStamped.transform.translation.y = collisionObj_pose.position.y;
			static_transformStamped.transform.translation.z = collisionObj_pose.position.z;
			static_transformStamped.transform.rotation.x = collisionObj_pose.orientation.x;
			static_transformStamped.transform.rotation.y = collisionObj_pose.orientation.y;
			static_transformStamped.transform.rotation.z = collisionObj_pose.orientation.z;
			static_transformStamped.transform.rotation.w = collisionObj_pose.orientation.w;
			static_broadcaster.sendTransform(static_transformStamped);
		}

	}

	// Now, let's add the collision object into the world
	// Little sleep necessary before adding it
	ros::Duration(0.2).sleep();
	// Add the remaining collision object
	if(perceptionAskedFlag_)
	{
		planning_scene_interface_.addCollisionObjects(collision_objects_vector_);
	}
}

void pr2perception::startPerception()
{
	perceptionAskedFlag_ = true;
}

void pr2perception::stopPerception()
{
	perceptionAskedFlag_ = false;
}

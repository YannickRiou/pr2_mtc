/**
 * \file pr2_gripper_event.cpp
 * \brief Handling of PR2 gripper force sensor and publish on topics 
 * \author Yannick R.
 * \version 0.1
 * \date 22/08/20
 *
 * Scout PR2 gripper force sensor and publish on "left_gripper_event" and "right_gripper_event" topics event.
 *
 */
#include <pr2_mtc/pr2_gripper_event.h>

struct sGripperPressure{
	int sum_leftFinger;
	int sum_rightFinger;

};

void onLeftGripperPressureCallback(const pr2_msgs::PressureStateConstPtr& msg, sGripperPressure& leftGripper)
{
	leftGripper.sum_leftFinger = 0;
	leftGripper.sum_rightFinger = 0;
	for(int i=0; i < 22; i++)
	{
		leftGripper.sum_leftFinger = leftGripper.sum_leftFinger + msg->l_finger_tip[i];
		leftGripper.sum_rightFinger = leftGripper.sum_rightFinger + msg->r_finger_tip[i];
	}
	leftGripper.sum_leftFinger = leftGripper.sum_leftFinger/22;
	leftGripper.sum_rightFinger = leftGripper.sum_rightFinger/22;
}


void onRightGripperPressureCallback(const pr2_msgs::PressureStateConstPtr& msg, sGripperPressure& rightGripper)
{
	rightGripper.sum_leftFinger = 0;
	rightGripper.sum_rightFinger = 0;
	for(int i=0; i < 22; i++)
	{
		rightGripper.sum_leftFinger = rightGripper.sum_leftFinger + msg->l_finger_tip[i];
		rightGripper.sum_rightFinger = rightGripper.sum_rightFinger + msg->r_finger_tip[i];
	}
	rightGripper.sum_leftFinger = rightGripper.sum_leftFinger/22;
	rightGripper.sum_rightFinger = rightGripper.sum_rightFinger/22;
}


/*void onLeftGripperAccelCallback(const pr2_msgs::AccelerometerStateConstPtr& msg, geometry_msgs::Vector3& leftGripper_accel)
{
	for (int i=0; i < msg->samples.size();i++)
	{
		leftGripper_accel.x = leftGripper_accel.x + msg->samples[i].x;
		leftGripper_accel.y = leftGripper_accel.y + msg->samples[i].y;
		leftGripper_accel.z = leftGripper_accel.z + msg->samples[i].z;
	}
	leftGripper_accel.x = leftGripper_accel.x/msg->samples.size();
	leftGripper_accel.y = leftGripper_accel.y/msg->samples.size();
	leftGripper_accel.z = leftGripper_accel.z/msg->samples.size();
}


void onRightGripperAccelCallback(const pr2_msgs::AccelerometerStateConstPtr& msg, geometry_msgs::Vector3& rightGripper_accel)
{
	for (int i=0; i < msg->samples.size();i++)
	{
		rightGripper_accel.x = rightGripper_accel.x + msg->samples[i].x;
		rightGripper_accel.y = rightGripper_accel.y + msg->samples[i].y;
		rightGripper_accel.z = rightGripper_accel.z + msg->samples[i].z;
	}
	rightGripper_accel.x = rightGripper_accel.x/msg->samples.size();
	rightGripper_accel.y = rightGripper_accel.y/msg->samples.size();
	rightGripper_accel.z = rightGripper_accel.z/msg->samples.size();
}*/


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_gripper_event");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	//geometry_msgs::Vector3 leftGripper_accel;
	//geometry_msgs::Vector3 rightGripper_accel;

	bool objectGrasped = false;

	sGripperPressure leftGripper;
    sGripperPressure rightGripper;

	sGripperPressure oldLeftGripper;
    sGripperPressure oldRightGripper;

	std_msgs::StringPtr str(new std_msgs::String);

	ros::Subscriber sub_gripper_left = nh.subscribe<pr2_msgs::PressureState>(PRESSURE_LEFT_SUB, 1, boost::bind(&onLeftGripperPressureCallback,_1,boost::ref(leftGripper)));
    ros::Subscriber sub_gripper_right = nh.subscribe<pr2_msgs::PressureState>(PRESSURE_RIGHT_SUB, 1, boost::bind(&onRightGripperPressureCallback,_1,boost::ref(rightGripper)));

	//ros::Subscriber sub_accel_left = nh.subscribe<pr2_msgs::AccelerometerState>(ACCEL_LEFT_SUB, 1,boost::bind(&onLeftGripperAccelCallback,_1,boost::ref(leftGripper_accel)));
	//ros::Subscriber sub_accel_right = nh.subscribe<pr2_msgs::AccelerometerState>(ACCEL_RIGHT_SUB, 1,boost::bind(&onRightGripperAccelCallback,_1,boost::ref(rightGripper_accel)));

	ros::Publisher left_gripper_event_pub = nh.advertise<std_msgs::String>("left_gripper_event", 5);
	ros::Publisher right_gripper_event_pub = nh.advertise<std_msgs::String>("right_gripper_event", 5);

	ros::Rate r(10); // 10 hz

	while(ros::ok())
	{

		/*if(leftGripper_accel.x > ACCEL_THRESHOLD)
		{		for (int i =0; i < 22; i++)
		{
			for (int j =0; j < 9; j++)
			{
				leftGripper.history_leftFinger[i][j+1] = leftGripper.history_leftFinger[i][j];
				leftGripper.history_rightFinger[i][j+1] = leftGripper.history_rightFinger[i][j];

				rightGripper.history_leftFinger[i][j+1] = rightGripper.history_leftFinger[i][j];
				rightGripper.history_rightFinger[i][j+1] = rightGripper.history_rightFinger[i][j];
			}

			leftGripper.history_leftFinger[i][0] = leftGripper.left_finger_tip[i];
			leftGripper.history_rightFinger[i][0] = leftGripper.right_finger_tip[i];


			rightGripper.history_leftFinger[i][0] = rightGripper.left_finger_tip[i];
			rightGripper.history_rightFinger[i][0] = rightGripper.right_finger_tip[i];
		}lish(str);
		}

		if(rightGripper_accel.x > ACCEL_THRESHOLD)
		{
			str->data = "human_take";
			right_gripper_event_pub.publish(str);
		}
		*/

		// Gripper is closed on something
		if(leftGripper.sum_leftFinger > PRESSURE_THRESHOLD && leftGripper.sum_rightFinger > PRESSURE_THRESHOLD)
		{
			if(!objectGrasped)
			{
				str->data = "object_grasped";
				left_gripper_event_pub.publish(str);
				objectGrasped = true;
				ROS_INFO("OBJECT GRASP ");
			}
			else if ((abs(leftGripper.sum_leftFinger - leftGripper.sum_rightFinger) > 1000) && objectGrasped)
			{
				str->data = "human_take";
				left_gripper_event_pub.publish(str);
				objectGrasped = false;
					ROS_INFO("HUMAN TAKE ");
			}
		}


		// Gripper is closed on something
		if(rightGripper.sum_leftFinger > PRESSURE_THRESHOLD && rightGripper.sum_rightFinger > PRESSURE_THRESHOLD)
		{
			if(!objectGrasped)
			{
				str->data = "object_grasped";
				right_gripper_event_pub.publish(str);
				objectGrasped = true;
			}
			else
			{
				str->data = "human_take";
				right_gripper_event_pub.publish(str);
				objectGrasped = false;
			}
		}



   		r.sleep();
	}

	return 0;
}

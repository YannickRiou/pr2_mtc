#include <pr2_msgs/PressureState.h>
#include <pr2_msgs/AccelerometerState.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include <ros/ros.h>

#define PRESSURE_LEFT_SUB "/pressure/l_gripper_motor"
#define PRESSURE_RIGHT_SUB "/pressure/r_gripper_motor"

#define ACCEL_LEFT_SUB "/accelerometer/l_gripper_motor"
#define ACCEL_RIGHT_SUB "/accelerometer/r_gripper_motor"

#define PRESSURE_THRESHOLD 3000
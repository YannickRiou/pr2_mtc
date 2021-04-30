#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <pr2_mtc/DockAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class Docking{
public:
    struct Params{
        double goalTolerance;
        double maxSpeed;
        double p;
        double i;
        double maxIntegral;
        ros::Duration controlPeriod;
        std::string tableFrame;
    };

    Docking(ros::NodeHandle& nh): nh_(nh), dockActionServer_(nh, "dock", boost::bind(&Docking::onDock, this, _1), false){
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 10, true);
        dockActionServer_.start();
    }

    void onDock(const pr2_mtc::DockGoalConstPtr& goal){
        ros::Time timeZero(0.0);
        Params params;
        getParams(params);
        
        pr2_mtc::DockResult asResult;
        if (!tfBuffer_.canTransform("map", "base_footprint", timeZero))
        {
            ROS_WARN_STREAM_NAMED("Docking", "Can't transform base_footprint frame in map frame. Aborting action.");
            asResult.error_code = asResult.NO_MAP_TO_FOOTPRINT_TRANSFORM;
            dockActionServer_.setAborted(asResult);
            return;
        }

        tf2::Transform poseTf; 

        poseTf.setOrigin( tf2::Vector3(goal->targetPose.pose.position.x, goal->targetPose.pose.position.y, goal->targetPose.pose.position.z) );
        poseTf.setRotation( tf2::Quaternion(goal->targetPose.pose.orientation.x, goal->targetPose.pose.orientation.y, goal->targetPose.pose.orientation.z, goal->targetPose.pose.orientation.w) );
      
        geometry_msgs::TransformStamped map2footprintMsg = tfBuffer_.lookupTransform("map", "base_footprint", timeZero);
        tf2::Transform map2footprint;
        tf2::fromMsg(map2footprintMsg.transform, map2footprint);

        double error = tf2::tf2Distance2(poseTf.getOrigin(), map2footprint.getOrigin());
        double cmd = 0.0, integral;
        ros::Time lastControl = ros::Time::now();
        ros::Time now = lastControl;
        ros::Time actionStart = ros::Time::now();
        geometry_msgs::Twist cmdVel;
        integral = 0.0;
        tf2::Transform footprint2pose;
        pr2_mtc::DockFeedback feedback;
        feedback.action_start = actionStart;
        feedback.distance_to_goal = std::sqrt(error);
        dockActionServer_.publishFeedback(feedback);
        while (std::abs(error) > params.goalTolerance && ros::ok())
        {
            map2footprintMsg = tfBuffer_.lookupTransform("map", "base_footprint", ros::Time(0.0));
            tf2::fromMsg(map2footprintMsg.transform, map2footprint);

            footprint2pose = map2footprint.inverse() * poseTf;

            error = std::pow(footprint2pose.getOrigin().getX(), 2) + std::pow(footprint2pose.getOrigin().getY(), 2);
            feedback.distance_to_goal = std::sqrt(error);

            dockActionServer_.publishFeedback(feedback);

            getParams(params);
            now = ros::Time::now();

            if (dockActionServer_.isPreemptRequested())
            {
                cmdVel.linear.x = 0.0;
                cmdVel.linear.y = 0.0;
                cmdVelPub_.publish(cmdVel);
                asResult.error_code = asResult.PREEMPTED;
                asResult.action_end = ros::Time::now();
                dockActionServer_.setPreempted(asResult);
                return;
            }

            integral = std::max(-params.maxIntegral, std::min(params.maxIntegral, integral + error * (now - lastControl).toSec()));

            cmd = std::max(-params.maxSpeed, std::min(params.maxSpeed, params.p * error + params.i * integral));

            double angle = std::atan2(footprint2pose.getOrigin().getY(), footprint2pose.getOrigin().getX());

            ROS_WARN_STREAM_NAMED("Docking", "ANGLE :" << angle);

            cmdVel.linear.x = cmd * std::cos(angle);
            cmdVel.linear.y = cmd * std::sin(angle);

            cmdVelPub_.publish(cmdVel);

            lastControl = now;
            params.controlPeriod.sleep();
        }
        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        asResult.action_end = ros::Time::now();
        cmdVelPub_.publish(cmdVel);
        asResult.error_code = asResult.SUCCESS;
        dockActionServer_.setSucceeded(asResult);
    }

    void getParams(Params& params)
    {
        double controlPeriod;
        nh_.param("goal_tolerance", params.goalTolerance, 0.2);
        params.goalTolerance *= params.goalTolerance;
        nh_.param("max_speed", params.maxSpeed, 0.2);
        nh_.param("P", params.p, 0.1);
        nh_.param("I", params.i, 0.05);
        nh_.param("max_integral", params.maxIntegral, 5.0);
        nh_.param("control_period", controlPeriod, 0.1);
        params.controlPeriod.fromSec(controlPeriod);
    }



protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    actionlib::SimpleActionServer<pr2_mtc::DockAction> dockActionServer_;
    ros::Publisher cmdVelPub_;

};

int main (int argc, char** argv){
    ros::init(argc, argv, "docking");
    ros::NodeHandle nh("~");
    Docking docking(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
}

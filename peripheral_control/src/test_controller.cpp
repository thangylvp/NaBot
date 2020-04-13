#include "header/controller.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <string.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_test");
	std::string odomTopic = "camera/odom/sample";
	std::string pathplnTopic = "xxx";
	std::string robotPoseTopic = "robot_pose";
	CONTROL::Controller main_controller(odomTopic, pathplnTopic, robotPoseTopic);

 	while (ros::ok())
 	{
		ROS_INFO("CUR POS -> x: [%f], y: [%f], z: [%f]", main_controller.curRobotPose.pose.position.x, main_controller.curRobotPose.pose.position.y, main_controller.curRobotPose.pose.position.z);
		// main_controller.updateTransform();

		switch (main_controller.curRState)
		{
		case CONTROL::ROBOTSTATE::IDLE : 
			main_controller.idle();
			break;
		case CONTROL::ROBOTSTATE::MOVING : 
			main_controller.moving();
			break;
		case CONTROL::ROBOTSTATE::TURNING :
			main_controller.turning();
			break;
		default:
			break;
		}
		main_controller.sendState();
		ros::spinOnce();
	}
	return 0;
}

#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>

nav_msgs::Odometry odom;
double current_RPY[3];
double current_Yaw;
enum RobotStage 
{
  IDLE=0,
  MOVING=1,
  TURNING=2
};

RobotStage current_stage = IDLE;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){  odom = *msg;}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_test");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe ("/camera/odom/sample", 1, odom_callback);
  ros::Rate loop_rate(50);

  double roll, pitch, yaw;

  while (ros::ok())
  {
    // ROS_INFO("Seq: [%d]", odom.header.seq);
    auto _orientation = odom.pose.pose.orientation;
    auto _position = odom.pose.pose.position;
    tf::Quaternion q(_orientation.x,_orientation.y,_orientation.z,_orientation.w);
    tf::Matrix3x3 m(q);    
    
    m.getRPY(current_RPY[0], current_RPY[1], current_RPY[2]);
    
    ROS_INFO("RPY -> r: [%f], p: [%f], y: [%f]", current_RPY[0], current_RPY[1], current_RPY[2]);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
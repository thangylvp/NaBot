#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_Cartographer_listener");

    ros::NodeHandle node;
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map", "base_link",ros::Time(0), transform);
            tf::Quaternion tfQuat = transform.getRotation(); // member fnc to extract the quaternion from a transform
            tf::Vector3 tfVec;  //tf-library type
            tfVec = transform.getOrigin(); // extract the vector from parent to child from transform
            geometry_msgs::Point pt; //equivalent geometry_msgs type
            pt.x = tfVec.getX(); //copy the components into geometry_msgs type
            pt.y = tfVec.getY();
            pt.z = tfVec.getZ();
            ROS_INFO("x: [%f], y: [%f], z: [%f]", pt.x, pt.y, pt.z);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        geometry_msgs::Twist vel_msg;
        rate.sleep();
    }
    return 0;
};
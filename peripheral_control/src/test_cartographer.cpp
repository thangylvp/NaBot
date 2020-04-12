#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_Cartographer_listener");

    ros::NodeHandle node;
    tf::TransformListener listener;

    ros::Rate rate(15.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map", "base_link",ros::Time(0), transform);

            tf::Vector3 tfVec = transform.getOrigin(); // extract the vector from parent to child from transform
            geometry_msgs::Point pt; //equivalent geometry_msgs type
            pt.x = tfVec.getX(); //copy the components into geometry_msgs type
            pt.y = tfVec.getY();
            pt.z = tfVec.getZ();
            ROS_INFO("x: [%f], y: [%f], z: [%f]", pt.x, pt.y, pt.z);

            tf::Quaternion tfQuat = transform.getRotation(); // member fnc to extract the quaternion from a transform
            geometry_msgs::Quaternion quat;  //geometry_msgs object for quaternion
            quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
            quat.y = tfQuat.y();
            quat.z = tfQuat.z();
            quat.w = tfQuat.w();
            ROS_INFO("Qx: [%f], Qy: [%f], Qz: [%f], Qw: [%f]", quat.x, quat.y, quat.z, quat.w);
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
//
// Created by hoangqc on 12/04/2020.
//
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include <vector>
#include "opencv2/opencv.hpp"

nav_msgs::OccupancyGrid curr_map;

bool need_check_map = false;

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    curr_map = *msg;
    nav_msgs::MapMetaData info = curr_map.info;

//    ROS_INFO("I heard: [%d x %d], res: [%f]",info.width, info.height, info.resolution);
//    ROS_INFO("Origin: [%f], [%f], [%f]", info.origin.position.x, info.origin.position.y, info.origin.position.z);
//    ROS_INFO("Data: [%d] ", curr_map.data.size());

    if(need_check_map)
    {
        cv::Mat map = cv::Mat(info.height, info.width, CV_8SC1, curr_map.data.data());
        cv::imshow("data", map);
        cv::waitKey(1);
    }

//    ROS_INFO()
//    map_pub.publish(msg->info.width);
}

void mapToWorld(int index, nav_msgs::MapMetaData info)
{
    int i = index / info.width;
    int j = index % info.width;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_converter");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("map", 10, mapConvert);
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
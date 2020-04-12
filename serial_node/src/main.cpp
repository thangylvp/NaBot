#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Int8.h"
#include <string.h>
#include <stdio.h>

serial::Serial ser;

enum ACTION {
    FORWARD = 0,
    LEFT = 1,
    BACK = 2,
    RIGHT = 3,
    ROTATE_LEFT = 4,
    ROTATE_RIGHT = 5,
    STOP = 6
};

void cmd_callback(const std_msgs::Int8::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing from serial port " <<  msg->data);
    if (msg->data == FORWARD){
        ser.write("u");
    }
    if (msg->data == LEFT){
        ser.write("l");
    } 
    if (msg->data == BACK){
        ser.write("d");
    } 
    if (msg->data == RIGHT){
        ser.write("d");
    } 
    if (msg->data == ROTATE_LEFT){
        ser.write("e");
    } 
    if (msg->data == ROTATE_RIGHT){
        ser.write("f");
    } 
    if (msg->data == STOP){
        ser.write("q");
    } 
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    std_msgs::String status;
    status.data = "ok";
     
    ros::Subscriber cmd_sub = nh.subscribe("cmd", 1000, cmd_callback);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("status", 1000);

    try
    {
        ser.setPort(argv[1]);
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

    while(ros::ok()){

        ros::spinOnce();
	    std::string tmp;
	    fflush(stdin);

        loop_rate.sleep();
    }
}

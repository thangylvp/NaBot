#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Int16.h"
#include <string.h>
#include <stdio.h>
#include <string>

serial::Serial ser;

std::string charToString(char a){
    std::string s = ""; 
    s = s + a;
    return s;
}

enum ACTION {
    STOP = 0,
    FORWARD = 1,
    RIGHT = 2,
    BACK = 3,
    LEFT = 4,
    ROTATE_LEFT = 5,
    ROTATE_RIGHT = 6,
    INTERRUPT = -1
};

void cmd_callback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing from serial port " <<  msg->data);
    if (msg->data == STOP){
        int f = 0;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == FORWARD){
        int f = 1;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    }
    if (msg->data == RIGHT){
        int f = 2;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == BACK){
        int f = 3;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == LEFT){
        int f = 4;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == ROTATE_LEFT){

        int f = 20;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == ROTATE_RIGHT){
        
        int f = -10;
        f = f*(-1) + 128;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 
    if (msg->data == INTERRUPT){
        
        int f = -1;
        char a = (char)f;
        std::string s = charToString(a);
        ser.write(s);
    } 

}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    std_msgs::String status;
    status.data = "ok";
     
    ros::Subscriber cmd_sub = nh.subscribe("control_command", 1000, cmd_callback);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("status", 1000);

    try
    {
        ser.setPort(argv[1]);
        ser.setBaudrate(115200);
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

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            status_pub.publish(result);
            ser.flush();
        }
        loop_rate.sleep();
    }
}

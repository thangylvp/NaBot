#include "ros/ros.h"
#include <string.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H
namespace CONTROL
{
    enum ROBOTSTATE {
        IDLE = 0,
        MOVING = 1,
        TURNING = 2
    };

    enum ACTION {
        FORWARD = 0,
        LEFT = 1,
        BACK = 2,
        RIGHT = 3,
        ROTATE_LEFT = 4,
        ROTATE_RIGHT = 5
    };

    class Controller{
    public:
        Controller(std::string odomTopic, std::string pathplnTopic);
        ~Controller();

        void sendCommand(ACTION actionType, float value);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        float odomToDistance(nav_msgs::Odometry curOdom, nav_msgs::Odometry preOdom);
        nav_msgs::Odometry curOdom, preOdom;
    private:
        ros::NodeHandle nh;
        ros::Subscriber subOdom;
        ros::Subscriber subPathpln;
        ros::Publisher pubCommand;
        std_msgs::Float32MultiArray command;
        
        float targetValue;
        bool newPathplnMessage = false;
        ACTION targetAction;
        ACTION intToAction[6] = {ACTION::FORWARD, ACTION::LEFT, ACTION::BACK, ACTION::RIGHT, ACTION::ROTATE_LEFT, ACTION::ROTATE_RIGHT};
        ROBOTSTATE curRState;
    };
}
#endif
#include "ros/ros.h"
#include <string.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int8.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"
#include <tf/transform_listener.h>

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
        ROTATE_RIGHT = 5,
        STOP = 6
    };

    class Controller{
    public:
        Controller(std::string odomTopic, std::string pathplnTopic, std::string robotPoseTopic);
        ~Controller();

        void sendCommand(ACTION actionType);
        void sendState();
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        float odomToDistance(nav_msgs::Odometry curOdom, nav_msgs::Odometry preOdom);
        float geomsgsToDistance(geometry_msgs::PoseStamped curRobotPose, geometry_msgs::PoseStamped preRobotPose);
        bool checkDoneMoving();
        bool checkDoneTurning();
        bool checkSTOP();
        void idle();
        void moving();
        void turning();
        void updateTransform();
        nav_msgs::Odometry curOdom, preOdom;
        int countRotate;
        bool newPathplnMessage = false;
        ROBOTSTATE curRState = ROBOTSTATE::IDLE;
        float targetValue;
        ACTION targetAction;
        geometry_msgs::PoseStamped curRobotPose, preRobotPose;

    private:
        ros::NodeHandle nh;
        ros::Subscriber subOdom;
        ros::Subscriber subPathpln;
        ros::Subscriber subRobotPose;
        ros::Publisher pubCommand;
        ros::Publisher pubState;
        std_msgs::Int8 command, rState;

        tf::TransformListener tf_listener;
        tf::StampedTransform transform_map_baselink;
        
        
        
        ACTION intToAction[7] = {ACTION::FORWARD, ACTION::LEFT, ACTION::BACK, ACTION::RIGHT, ACTION::ROTATE_LEFT, ACTION::ROTATE_RIGHT, ACTION::STOP};
        
    };
}
#endif
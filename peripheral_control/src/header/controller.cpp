#include "controller.h"

CONTROL::Controller::Controller(std::string odomTopic, std::string pathplnTopic, std::string robotPoseTopic) {
    subOdom = nh.subscribe(odomTopic, 1, &Controller::odomCallback, this);
    subPathpln = nh.subscribe(pathplnTopic, 1, &Controller::pathplnCallback, this);
    subRobotPose = nh.subscribe(robotPoseTopic, 1, &Controller::robotPoseCallback, this);
    pubCommand = nh.advertise<std_msgs::Int16>("control_command", 1);
    pubState = nh.advertise<std_msgs::Int8>("robot_state", 1);
}
CONTROL::Controller::~Controller() {
    
}

void CONTROL::Controller::sendCommand(ACTION actionType) {
    command.data = actionType;
    std::cerr << "SEND ACTION : " << actionType << std::endl;
    pubCommand.publish(command);
}

void CONTROL::Controller::sendState() {
    // std::cerr << "curRState : " << curRState << std::endl;
    int dat = (curRState == ROBOTSTATE::IDLE) ? 1 : 0;
    // std::cerr << dat << std::endl;
    rState.data = ((curRState == ROBOTSTATE::IDLE) ? 1 : 0);
    pubState.publish(rState);
}

void CONTROL::Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    curOdom = *msg;
}

void CONTROL::Controller::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curRobotPose = *msg;
}

void CONTROL::Controller::pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    targetAction = intToAction[(int)msg->data[0]];
    targetValue = msg->data[1];
    std::cerr << "New request " << targetAction << " " << targetValue << std::endl;
    newPathplnMessage = true;
}

void CONTROL::Controller::idle() {
    if (newPathplnMessage) {
        newPathplnMessage = false;
        // preOdom = curOdom;
        preRobotPose = curRobotPose;
        countRotate = 0;
        if (targetAction == ACTION::STOP || targetAction == ACTION::INTERRUPT) {
            return;
        }
        if (targetAction == ACTION::FORWARD || targetAction == ACTION::BACK || targetAction == ACTION::LEFT || targetAction == ACTION ::RIGHT) 
            curRState = ROBOTSTATE::MOVING;
        
        if (targetAction == ACTION::ROTATE_LEFT || targetAction == ACTION::ROTATE_RIGHT)
            curRState = ROBOTSTATE::TURNING;
        
    }
}

void CONTROL::Controller::moving() {
    // std::cerr << "Moving" << std::endl;
    if (checkSTOP()) {
        curRState = ROBOTSTATE::IDLE;
        sendCommand(ACTION::STOP);
        return;
    }
    if (checkDoneMoving()) {
        curRState = ROBOTSTATE::IDLE;
        sendCommand(ACTION::STOP);
        return;
    }
    sendCommand(targetAction);
    // moving
}

void CONTROL::Controller::turning() {
    if (checkSTOP()) {
        curRState = ROBOTSTATE::IDLE;
        sendCommand(ACTION::STOP);
        return;
    }

    if (checkDoneTurning()) {
        curRState = ROBOTSTATE::IDLE;
        sendCommand(ACTION::STOP);
        return;
    }
    sendCommand(targetAction);
    sendCommand(ACTION::STOP);
    countRotate++;
    // turning
}

bool CONTROL::Controller::checkDoneMoving() {
    float distance = fabs(geomsgsToDistance(curRobotPose, preRobotPose) - targetValue);
    // std::cerr << "DISTANCE " << distance << std::endl;
    if (distance < 0.05) 
        return true;
    return false;
}

bool CONTROL::Controller::checkDoneTurning() {
    /*
    if (countRotate * 10 >= targetValue)
        return true;
    */
    return false;
}

bool CONTROL::Controller::checkSTOP() {
    if (newPathplnMessage) {
        newPathplnMessage = false;
        if (targetAction == ACTION::STOP) {
            return true;
        }
    }
    return false;
    
}

void CONTROL::Controller::updateTransform()
{
    try{
        tf_listener.lookupTransform("map", "base_link",ros::Time(0), transform_map_baselink);

//        tf::Vector3 tfVec = transform_map_baselink.getOrigin();
//        geometry_msgs::Point pt;
//        pt.x = tfVec.getX();pt.y = tfVec.getY();pt.z = tfVec.getZ();
//        ROS_INFO("x: [%f], y: [%f], z: [%f]", pt.x, pt.y, pt.z);

//        tf::Quaternion tfQuat = transform_map_baselink.getRotation();
//        geometry_msgs::Quaternion quat;
//        quat.x = tfQuat.x();quat.y = tfQuat.y();quat.z = tfQuat.z();quat.w = tfQuat.w();
//        ROS_INFO("Qx: [%f], Qy: [%f], Qz: [%f], Qw: [%f]", quat.x, quat.y, quat.z, quat.w);

    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}

float CONTROL::Controller::odomToDistance(nav_msgs::Odometry curOdom, nav_msgs::Odometry preOdom) {
    
    float dx = curOdom.pose.pose.position.x - preOdom.pose.pose.position.x;
    float dy = curOdom.pose.pose.position.y - preOdom.pose.pose.position.y;
    float dz = curOdom.pose.pose.position.z - preOdom.pose.pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float CONTROL::Controller::geomsgsToDistance(geometry_msgs::PoseStamped curRobotPose, geometry_msgs::PoseStamped preRobotPose) {
    float dx = curRobotPose.pose.position.x - preRobotPose.pose.position.x;
    float dy = curRobotPose.pose.position.y - preRobotPose.pose.position.y;
    float dz = curRobotPose.pose.position.z - preRobotPose.pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

#include "controller.h"

CONTROL::Controller::Controller(std::string odomTopic, std::string pathplnTopic) {
    subOdom = nh.subscribe(odomTopic, 1, &Controller::odomCallback, this);
    subPathpln = nh.subscribe(pathplnTopic, 1, &Controller::pathplnCallback, this);
    pubCommand = nh.advertise<std_msgs::Int8>("control_command", 10);
    pubState = nh.advertise<std_msgs::Int8>("robot_state", 10);
}
CONTROL::Controller::~Controller() {
    
}

void CONTROL::Controller::sendCommand(ACTION actionType) {
    command.data = actionType;
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

void CONTROL::Controller::pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    targetAction = intToAction[(int)msg->data[0]];
    targetValue = msg->data[1];
    std::cerr << "New request " << targetAction << " " << targetValue << std::endl;
    newPathplnMessage = true;
}

void CONTROL::Controller::idle() {
    if (newPathplnMessage) {
        newPathplnMessage = false;
        preOdom = curOdom;
        countRotate = 0;
        if (targetAction < 4) 
            curRState = ROBOTSTATE::MOVING;
        else 
        if (targetAction < 6)
            curRState = ROBOTSTATE::TURNING;
        else 
            curRState = ROBOTSTATE::IDLE;
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
    float distance = fabs(odomToDistance(curOdom, preOdom) - targetValue);
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
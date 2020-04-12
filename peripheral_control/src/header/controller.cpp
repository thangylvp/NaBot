#include "controller.h"

CONTROL::Controller::Controller(std::string odomTopic, std::string pathplnTopic) {
    subOdom = nh.subscribe(odomTopic, 1, &Controller::odomCallback, this);
    subPathpln = nh.subscribe(pathplnTopic, 1, &Controller::pathplnCallback, this);
    pubCommand = nh.advertise<std_msgs::Int8>("control_command", 1000);
    pubState = nh.advertise<std_msgs::Int8>("robot_state", 1000);
}
CONTROL::Controller::~Controller() {
    
}

void CONTROL::Controller::sendCommand(ACTION actionType) {
    command.data = actionType;
    pubCommand.publish(command);
}

void CONTROL::Controller::sendState() {
    rState.data = ((curRState == ROBOTSTATE::IDLE) ? 1 : 0);
    pubState.publish(rState);
}

void CONTROL::Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    curOdom = *msg;
}

void CONTROL::Controller::pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::cerr << "New request" << std::endl;
    targetAction = intToAction[(int)msg->data[0]];
    targetValue = msg->data[1];
    newPathplnMessage = true;
}

void CONTROL::Controller::idle() {
    if (newPathplnMessage) {
        newPathplnMessage = false;
        preOdom = curOdom;
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

float CONTROL::Controller::odomToDistance(nav_msgs::Odometry curOdom, nav_msgs::Odometry preOdom) {
    float dx = curOdom.pose.pose.position.x - preOdom.pose.pose.position.x;
    float dy = curOdom.pose.pose.position.y - preOdom.pose.pose.position.y;
    float dz = curOdom.pose.pose.position.z - preOdom.pose.pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
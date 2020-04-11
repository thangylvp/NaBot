#include "controller.h"

CONTROL::Controller::Controller(std::string odomTopic, std::string pathplnTopic) {
    subOdom = nh.subscribe(odomTopic, 1, &Controller::odomCallback, this);
    subPathpln = nh.subscribe(pathplnTopic, 1, &Controller::pathplnCallback, this);
    pubCommand = nh.advertise<std_msgs::Float32MultiArray>("array", 10);
}
CONTROL::Controller::~Controller() {
    
}

void CONTROL::Controller::sendCommand(ACTION actionType, float value) {
    command.data.clear();
    command.data.push_back((float)actionType);
    command.data.push_back(value);
    pubCommand.publish(command);
}

void CONTROL::Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    curOdom = *msg;
}

void CONTROL::Controller::pathplnCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    targetAction = intToAction[(int)msg->data[0]];
    targetValue = msg->data[1];
    newPathplnMessage = true;
}


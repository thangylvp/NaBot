#include "header/controller.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <string.h>

#include <string.h>
#include <termios.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <fstream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"


int getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int robotState;
bool newMessage = false;
void chatter9(const std_msgs::Int8::ConstPtr& msg)
{
    newMessage = true;
    robotState = msg->data; 
    // std::cerr << "------------------" << robotState << std::endl;
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
int numAction = 0;
ACTION listAction[1000] = {ACTION::STOP};
float distance[1000] = {0};
std::map<std::string, ACTION> charToAction;
//ACTION listAction[1] = {ACTION::FORWARD};
//float distance[1] = {0.2};
void initCharToAction() {
    charToAction["F"] = ACTION::FORWARD;
    charToAction["R"] = ACTION::RIGHT;
    charToAction["B"] = ACTION::BACK;
    charToAction["L"] = ACTION::LEFT;
}

void loadMission(std::string pathToFile) {
    std::cerr << "PATH NAME " << pathToFile << std::endl;
    std::ifstream ifs(pathToFile, std::ifstream::in);
    ifs >> numAction;
    std::string action;
    float value;
    for (int i = 0; i < numAction; i++) {
        ifs >> action >> value;
        listAction[i] = charToAction[action];
        distance[i] = value;
    }
}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "test1");
    ros::NodeHandle n;
	ros::Subscriber subState = n.subscribe("robot_state", 1, chatter9);
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("xxx", 1);
    initCharToAction();
    loadMission(std::string(argv[1]));
    for (int i = 0; i < numAction; i++) {
        std::cerr << "action : " << listAction[i] << " " << distance[i] << std::endl;
    }
    std_msgs::Float32MultiArray array;
    bool run = true;
    int curStep = 0;
    ros::Rate r(10);
 	while (ros::ok())
 	{
		// ROS_INFO("CUR POS -> x: [%f], y: [%f], z: [%f]", main_controller.curOdom.pose.pose.position.x, main_controller.curOdom.pose.pose.position.y, main_controller.curOdom.pose.pose.position.z);
		char key = getkey();
        if (key == 's'){
            array.data.clear();
            array.data.push_back(ACTION::STOP);
            array.data.push_back(0.0);
            std::cerr << array.data[0] <<  " " << array.data[1] << std::endl;
            pub.publish(array);
            run = false;
        }

        if (run == true) {
            if (newMessage == true) {
                newMessage = false;
                if (robotState == 1) {
                    if (curStep < numAction) {
                        array.data.clear();
                        array.data.push_back(listAction[curStep]);
                        array.data.push_back(distance[curStep]);
                        std::cerr << "SEND " << array.data[0] <<  " " << array.data[1] << std::endl;
                        pub.publish(array);
                        curStep++;
                    } else 
                        break;
                }
            }
        }
        r.sleep();
        ros::spinOnce();
	}
	return 0;
}

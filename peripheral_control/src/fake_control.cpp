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

void chatter9(const std_msgs::Int8::ConstPtr& msg)
{
    robotState = msg->data; 
    // std::cerr << "------------------" << robotState << std::endl;
}


enum ACTION {
        FORWARD = 0,
        LEFT = 1,
        BACK = 2,
        RIGHT = 3,
        ROTATE_LEFT = 4,
        ROTATE_RIGHT = 5,
        STOP = 6
    };

int main(int argc, char **argv)
{

	ros::init(argc, argv, "fake_control");
    ros::NodeHandle n;
	ros::Subscriber subState = n.subscribe("robot_state", 1, chatter9);
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("xxx", 10);
    std_msgs::Float32MultiArray array;
 	while (ros::ok())
 	{
		// ROS_INFO("CUR POS -> x: [%f], y: [%f], z: [%f]", main_controller.curOdom.pose.pose.position.x, main_controller.curOdom.pose.pose.position.y, main_controller.curOdom.pose.pose.position.z);
		char key = getkey();
        if (key == 'u') {
            array.data.clear();
            array.data.push_back(ACTION::FORWARD);
            array.data.push_back(1.0);
            std::cerr << array.data[0] <<  " " << array.data[1] << std::endl;
            pub.publish(array);
        }
        if (key == 's'){
            array.data.clear();
            array.data.push_back(ACTION::STOP);
            array.data.push_back(0.0);
            std::cerr << array.data[0] <<  " " << array.data[1] << std::endl;
            pub.publish(array);
        }
        if (key == 'l'){
            array.data.clear();
            array.data.push_back(ACTION::ROTATE_LEFT);
            array.data.push_back(50.0);
            std::cerr << array.data[0] <<  " " << array.data[1] << std::endl;
            pub.publish(array);
        }
        if (key == 'r'){
            array.data.clear();
            array.data.push_back(ACTION::ROTATE_RIGHT);
            array.data.push_back(10.0);
            std::cerr << array.data[0] <<  " " << array.data[1] << std::endl;
            pub.publish(array);
        }
        ros::spinOnce();
	}
	return 0;
}

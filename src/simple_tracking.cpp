#include <ros/ros.h>

#include "tracker.hpp"

#include <stdlib.h>


void voiceCb(const std_msgs::String::ConstPtr& msg) {
	std::string cmd = msg->data;
	if (cmd.compare("follow me") == 0 || cmd.compare("follow") == 0 || cmd.compare("start") == 0) {

	} else if (cmd.compare("stop") == 0 || cmd.compare("halt") == 0) {

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_tracking");
    ros::NodeHandle nh;
    ros::Subscriber voiceSub;

	Tracker tracker("Differential");

    voiceSub = nh.subscribe("/recognizer_1/output", 1, voiceCb);
	ros::spin();

	return 0;
}

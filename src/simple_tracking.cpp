#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "tracker.hpp"

#include <stdlib.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>



void voiceCb(const std_msgs::String::ConstPtr& msg) {
	std::string cmd = msg->data;
	if (cmd.compare("follow me") == 0 || cmd.compare("follow") == 0 || cmd.compare("start") == 0) {

	} else if (cmd.compare("stop") == 0 || cmd.compare("halt") == 0) {

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_tracking");
//	ros::NodeHandle nh;
//	ros::Subscriber voiceSub;

	Tracker tracker("Differential");

//	voiceSub = nh.subscribe("/recognizer_1/output", 1, voiceCb);

	ros::spin();

//	ros::Rate loopRate(100);
//	while (ros::ok()) {
//		ros::spinOnce();
//		loopRate.sleep();
//	}

	return 0;
}

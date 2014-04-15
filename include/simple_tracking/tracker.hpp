/*
 * tracker.hpp
 *
 *  Created on: Apr 15, 2014
 *      Author: spacemaster08
 */

#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <stdlib.h>
#include <string>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "kalman_filter.hpp"
#include "segmentation.hpp"
#include "path.hpp"

#define MIN_FOLLOW_THRESHOLD 	0.40f		// min distance in meters
#define MAX_FOLLOW_THRESHOLD 	2.0f		// max distance in meters
#define STEP_FOLLOW_THRESHOLD	0.1f		// step size if target is lost


#define MIN_FOV					2500		// std angular field of view in voxels around theta
#define MAX_FOV				 	6000		// max angular field of view in voxels around theta
#define STEP_FOV				100			// step size if target is lost

struct Position{
	float x, y, theta;
};

class Tracker {
private:
	KalmanFilter kf;
	Path path;

	Position pos;

	int fov;
	float threshold;

	bool firstrun, ackermann;
	bool track, follow, marker, slize, person;

    ros::NodeHandle nh_;
    ros::Subscriber pclSub;
    ros::Publisher slizePub, personPub, velCmdPub, ackCmdPub, markerPub;

    ros::ServiceServer startService, stopService;




public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Tracker(std::string type);
	virtual ~Tracker();

	void cloudCb (const sensor_msgs::PointCloud2::ConstPtr& msg);
	bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

#endif /* TRACKER_HPP_ */

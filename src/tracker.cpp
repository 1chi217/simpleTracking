/*
 * tracker.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: spacemaster08
 */

#include <tracker.hpp>

Tracker::Tracker(std::string type) :
	kf(0.1,0.01,0.01,10,0,0),
	pclSub(nh_.subscribe("velodyne_points", 5, &Tracker::cloudCb, this)),
	slizePub(nh_.advertise<sensor_msgs::PointCloud2> ("/tracking/velodyne_slize",1)),
	personPub(nh_.advertise<sensor_msgs::PointCloud2> ("/tracking/velodyne_person",1)),
	velCmdPub(nh_.advertise<geometry_msgs::Twist> ("/tracking/vel_cmd",1)),
	ackCmdPub(nh_.advertise<ackermann_msgs::AckermannDrive> ("/tracking/ackermann_cmd",1)),
	markerPub(nh_.advertise<visualization_msgs::Marker>( "/tracking/marker",1)),
	startService(nh_.advertiseService("/tracking/start_tracking", &Tracker::start, this)),
	stopService(nh_.advertiseService("/tracking/stop_tracking", &Tracker::stop, this))
{
	pos.x = 0;
	pos.y = 0;
	pos.theta = 0;

	threshold = FLT_MAX;
	fov = MIN_FOV;

	track = false;
	follow = false;

	marker = false;
	slize = false;
	person = false;

	firstrun = true;

	if(type.compare("Ackermann") == 0 ){
		ackermann = true;
	} else {
		ackermann = false;
	}
	ROS_INFO("I'm alive");

}

Tracker::~Tracker() {
	// TODO Auto-generated destructor stub
}

bool Tracker::start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	pos.x = 0;
	pos.y = 0;
	pos.theta = 0;

	threshold = FLT_MAX;
	fov = MIN_FOV;

	track = true;
	follow = true;
	firstrun = true;
	person = true;
	marker = true;
	slize = true;
	return true;
}

bool Tracker::stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
	track = false;
	follow = false;
	marker = false;
	slize = false;
	person = false;
	return true;
}

void Tracker::cloudCb (const sensor_msgs::PointCloud2::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZ> inPCL;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inPCLptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPCLptr (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr personPCL (new pcl::PointCloud<pcl::PointXYZI>);
	if(track){

		// get pcl::PointCloud<pcl::PointXYZ>::Ptr from pcl ROS msg
		pcl::fromROSMsg(*msg, inPCL);
		inPCLptr = inPCL.makeShared();

		// Segmentation seg(3000, 20, 0.50);		// Velodyne mounted on car
		Segmentation seg(6000, 500, 0.30);			// Velodyne mounted on pioneer
		seg.setInputCloud(inPCLptr);
		seg.cutOutSector(pos.theta, fov, filteredPCLptr);
		seg.removeFloor(filteredPCLptr);
		seg.cluster(personPCL);

		std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
		centroids = seg.getCentroids();

		if (centroids.size() > 0){
			int min = 0;
			float minDist = FLT_MAX;
			for(unsigned int i = 0; i < centroids.size(); i++){
				float dist = sqrt(pow((float) (centroids.at(i)[0] - pos.x),2) + pow((float) (centroids.at(i)[1]  - pos.y),2));
				if(dist < minDist){
					 minDist = dist;
					 min = i;
				}
			}

			if(minDist < threshold){

				 if(firstrun){
					 firstrun = false;
					 threshold = MIN_FOLLOW_THRESHOLD;
					 kf.setPosition((float) centroids.at(min)[0], (float) centroids.at(min)[1]);
				 }
				 kf.process((float) centroids.at(min)[0], (float) centroids.at(min)[1]);

				 Eigen::Vector4f newPos = kf.getState();
				 pos.x = newPos[0];
				 pos.y = newPos[1];

				 ROS_INFO("Pos %f %f", pos.x, pos.y);

				 pos.theta = atan2(pos.y, pos.x);
				 if(pos.theta < 0){
					 pos.theta += M_2_PI;
				 }

				 threshold = MIN_FOLLOW_THRESHOLD;
				 fov = MIN_FOV;

				if(follow){
					path.update(pos.x, pos.y, ros::Time::now());
					if(ackermann){

					} else {
						geometry_msgs::Twist vel_cmd = path.getCmdDiff();
						ROS_INFO("l %f a %f", vel_cmd.linear.x, vel_cmd.angular.z);
						velCmdPub.publish(vel_cmd);
					}
				}

			} else {
				 if(threshold < MAX_FOLLOW_THRESHOLD)
					 threshold += STEP_FOLLOW_THRESHOLD;
				 if(fov < MAX_FOV)
					 fov += STEP_FOV;
			}
		}
	}




	if(marker){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "velodyne";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = pos.x;
		marker.pose.position.y = pos.y;
		marker.pose.position.z = 1;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		markerPub.publish( marker );
	}

	if(slize){
		sensor_msgs::PointCloud2 pubMsg;
		pcl::toROSMsg(*(filteredPCLptr.get()), pubMsg);
		pubMsg.header = msg->header;
		slizePub.publish(pubMsg);
	}

	if(person){
		sensor_msgs::PointCloud2 pubMsg;
		pcl::toROSMsg(*(personPCL.get()), pubMsg);
		pubMsg.header = msg->header;
		personPub.publish(pubMsg);
	}
};



#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "kalman_filter.hpp"
#include "segmentation.hpp"
#include "path.hpp"


#include <tf/transform_listener.h>
#include <tf/tf.h>

#define THRESHOLD 0.40f

sensor_msgs::PointCloud2::ConstPtr PclMsg;

bool recPcl;

void cloudCb (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    PclMsg = msg;
    recPcl = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_tracking");
    ros::NodeHandle nh;
    ros::Subscriber pclSub;
    ros::Publisher slizePub, personPub, cmdPub;

//    tf::TransformListener listener;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    pclSub = nh.subscribe("velodyne_points", 1, cloudCb);
    slizePub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_slize",1);
    personPub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_person",1);
    cmdPub = nh.advertise<geometry_msgs::Twist> ("follow_cmd",1);

    sensor_msgs::PointCloud2 msg;
    std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
    Eigen::Vector4f person;

    pcl::PointCloud<pcl::PointXYZ> in;
    ros::Rate loopRate(100);


//    float angle = M_PI;
    float angle = 0;
    float threshold = THRESHOLD;
    bool firstrun = true;
    int fov = 2500;

    KalmanFilter kf(0.1,0.01,0.01,10,0,0);
    Path path;

    while(ros::ok()){

        // if we have received a new pointcloud
        if(recPcl){
//            ROS_INFO("New pointcloud");
            recPcl = false;
            pcl::PointCloud<pcl::PointXYZI>::Ptr publish (new pcl::PointCloud<pcl::PointXYZI>);
            // Read in the cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                    cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
            // get pcl::PointCloud<pcl::PointXYZ>::Ptr from pcl ROS msg
            pcl::fromROSMsg(*PclMsg, in);
            cloud = in.makeShared();

//            Segmentation seg(3000, 20, 0.50);
            Segmentation seg(6000, 500, 0.30);
            seg.setInputCloud(cloud);
            seg.cutOutSector(angle, fov, cloud_f);
            seg.removeFloor(cloud_f);
            seg.cluster(publish);

            centroids = seg.getCentroids();
//            ROS_INFO("Detected %d possible objects", centroids.size());

            if(firstrun){
                if (centroids.size() > 0){
                    firstrun = false;
                    int min = 0;
                    float minDist = 1000;
                    for(unsigned int i = 0; i < centroids.size(); i++){
                        float dist = sqrt(pow(centroids.at(i)[0],2) + pow(centroids.at(i)[1],2));
                        if(dist < minDist){
                            minDist = dist;
                            min = i;
                        }
                    }
//                    ROS_INFO("Tracking starts at %f,%f",centroids.at(min)[0], centroids.at(min)[1]);
                    kf.setPosition(centroids.at(min)[0], centroids.at(min)[1]);
                    kf.process(centroids.at(min)[0], centroids.at(min)[1]);
                    person = kf.getState();
                    angle = atan2(person[1], person[0]);
                    if(angle < 0){
                        angle += 2 * M_PI;
                    }
                }
            } else {
                if (centroids.size() > 0){
                    int min = 0;
                    float minDist = 1000;
                    for(unsigned int i = 0; i < centroids.size(); i++){
                        float dist = sqrt(pow(centroids.at(i)[0] - person[0],2) + pow(centroids.at(i)[1]  - person[1],2));
                        if(dist < minDist){
                            minDist = dist;
                            min = i;
                        }
                    }

                    if(minDist < threshold){
//                        ROS_INFO("Person at %f,%f",centroids.at(min)[0], centroids.at(min)[1]);

//                        geometry_msgs::PointStamped point, global;
//                        point.point.x = centroids.at(min)[0];
//                        point.point.y = centroids.at(min)[1];
//                        point.header.frame_id = "velodyne";
//                        point.header.stamp = PclMsg->header.stamp;

//                        try {
//                            listener.waitForTransform(point.header.frame_id, "odom", point.header.stamp, ros::Duration(0.2));
//                            listener.transformPoint("odom", point, global);
//                        } catch (tf::TransformException& ex) {
//                            ROS_ERROR("Invaild transform %s", ex.what());
//                        }
//                        ROS_INFO("Global at %f,%f",global.point.x, global.point.y);

                        kf.process(centroids.at(min)[0], centroids.at(min)[1]);
                        person = kf.getState();
                        angle = atan2(person[1], person[0]);
                        if(angle < 0){
                            angle += 2 * M_PI;
                        }
                        threshold = THRESHOLD;
                        fov = 2500;



                    } else {
                        if(threshold < 2)
                            threshold += 0.1;
                        if(fov < 6000)
                            fov += 100;
                    }
                }
            }

            path.update(person[0], person[1], ros::Time::now());
            geometry_msgs::Twist vel_cmd = path.getCmdDiff();

            ROS_INFO("l %f a %f", vel_cmd.linear.x, vel_cmd.angular.z);
            cmdPub.publish(vel_cmd);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = person[0];
            marker.pose.position.y = person[1];
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
            vis_pub.publish( marker );

            pcl::toROSMsg(*(cloud_f.get()), msg);
            msg.header = PclMsg->header;
            slizePub.publish(msg);

            pcl::toROSMsg(*(publish.get()), msg);
            msg.header = PclMsg->header;
            personPub.publish(msg);

        } else {
//            ROS_WARN("NO PCL");
        }
        ros::spinOnce();
        loopRate.sleep();
    }
    ROS_WARN("should not be here");
    return 0;
}

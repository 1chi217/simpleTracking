/*
 * Segmentation.cpp
 *
 *  Created on: 24.03.2014
 *      Author: Michael
 */
#include <ros/ros.h>
#include "segmentation.hpp"


Segmentation::Segmentation(int maxClusterSize, int minClusterSize, double clusterTolerance){

    this->maxClusterSize = maxClusterSize;
    this->minClusterSize = minClusterSize;
    this->clusterTolerance = clusterTolerance;
}

Segmentation::~Segmentation() {

}

int Segmentation::findIdx(double goal){

    double angle, distCCW, distCW;
    double minAngle, cenAngle, maxAngle;
    int pos = 0, step = cloud->points.size()/4;

    if(goal < 0)
        goal =  2 * M_PI + goal;
//    ROS_INFO("goal %f", goal / M_PI * 180);

    minAngle = atan2(cloud->at(pos).y, cloud->at(pos).x);
    if(minAngle < 0)
        minAngle = 2 * M_PI + minAngle;

    pos = cloud->points.size()/2;
    cenAngle = atan2(cloud->at(pos).y, cloud->at(pos).x);
    if(cenAngle < 0)
        cenAngle = 2 * M_PI + cenAngle;

    pos = cloud->points.size() -1 ;
    maxAngle = atan2(cloud->at(pos).y, cloud->at(pos).x);
    if(maxAngle < 0)
        maxAngle = 2 * M_PI + maxAngle;

//    ROS_INFO("min %f, cen %f, max %f", minAngle / M_PI * 180, cenAngle / M_PI * 180, maxAngle / M_PI * 180);

    pos = cloud->points.size()/2;
    angle = cenAngle;
//    ROS_INFO("Search angle");
    while(fabs(angle - goal) > 0.01){

        if(goal < angle){
            distCW = angle - goal;
            distCCW = 2 * M_PI - distCW;
        } else {
            distCCW = goal - angle;
            distCW = 2 * M_PI - distCCW;
        }

        if(distCCW < distCW){
            pos -= step;
        } else {
            pos += step;
        }
        step /= 2;
        if(step == 1){
            if(fabs(angle - goal) > 0.01){
//                ROS_WARN("Angle can not be found!");
                return -1;
            }
        }
        angle = atan2(cloud->at(pos).y, cloud->at(pos).x);
        if(angle < 0)
            angle =  2 * M_PI + angle;
    }
//    ROS_INFO("angle %f, pos %d", angle/ M_PI * 180, pos);
//    ROS_INFO("");
    return pos;
}

void Segmentation::cutOutSector(double angle, int fov, pcl::PointCloud<pcl::PointXYZ>::Ptr out){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    int idx = findIdx(angle);
    if(idx != -1 && idx > (fov+1) && idx < cloud->points.size() - (fov+1)){
        for(int i = idx - fov; i < idx + fov; i++){
            cloud_f->push_back(cloud->at(i));
        }
    } /*else {
        ROS_WARN("Out of Boundaries");
    }*/
    *cloud = *cloud_f;
    *out = *cloud;
}

void Segmentation::removeFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr out){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
    seg.setEpsAngle(15 / 180 * M_PI);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);

    int nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
    *out = *cloud;
}


void Segmentation::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr out){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.30); // 2cm
    ec.setMinClusterSize (25);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector4f min;
        Eigen::Vector4f max;
        Eigen::Vector4f centroid;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        pcl::getMinMax3D (*cloud_cluster, min, max);
        pcl::compute3DCentroid (*cloud_cluster, centroid);
        if((max[2] - min[2]) < 2.0 && (max[1] - min[1]) < 1.2 && (max[0] - min[0]) < 1.2){
            if(centroid[2] < 0 && centroid[2] > -1.60 &&
                    sqrt((double)(centroid[0]*centroid[0] +  centroid[1]*centroid[1])) < 10){
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                    pcl::PointXYZI intensityP;
                    pcl::PointXYZ oldP = cloud->points[*pit];
                    intensityP.x = oldP.x;
                    intensityP.y = oldP.y;
                    intensityP.z = oldP.z;
                    intensityP.intensity = j;
                    cloud_f->points.push_back(intensityP); //*
                }
                centroids.push_back(centroid);
            }
            j += 15;
        }
    }
    *out = *cloud_f;
}

void Segmentation::process(double angle, int fov,pcl::PointCloud<pcl::PointXYZI>::Ptr out){
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
    cutOutSector(angle, fov, tmp);
    removeFloor(tmp);
    cluster(out);
}

std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >  Segmentation::getCentroids(){
    return centroids;
}

void Segmentation::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in){
    this->cloud = in;
}

void Segmentation::setMaxClusterSize(int maxClusterSize){
    this->maxClusterSize = maxClusterSize;
}

void Segmentation::setMinClusterSize(int minClusterSize){
    this->minClusterSize = minClusterSize;
}

void Segmentation::setClusterTolerance(double clusterTolerance){
    this->clusterTolerance = clusterTolerance;
}

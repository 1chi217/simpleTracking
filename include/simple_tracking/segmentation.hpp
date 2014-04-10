/*
 * Segmentation.hpp
 *
 *  Created on: 24.03.2014
 *      Author: Michael
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <stdlib.h>

class Segmentation {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
    int maxClusterSize;
    int minClusterSize;
    float clusterTolerance;

    int findIdx(float angle);

public:
    Segmentation(int maxClusterSize = 2500, int minClusterSize = 25, int clusterTolerance = 0.30);
    virtual ~Segmentation();

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in);
    void cutOutSector(float angle, int fov, pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    void removeFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    void cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void process(float angle, int fov, pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >  getCentroids();

    void setMaxClusterSize(int maxClusterSize);
    void setMinClusterSize(int minClusterSize);
    void setClusterTolerance(float clusterTolerance);

};

#endif /* SEGMENTATION_H_ */

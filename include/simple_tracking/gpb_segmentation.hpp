/*
 * GPB_Segmentation.hpp
 *
 *  Created on: 24.04.2014
 *      Author: Michael
 */

#ifndef GPB_SEGMENTATION_H_
#define GPB_SEGMENTATION_H_

#include "segmentation.hpp"

#include <boost/array.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define N               100         // number of bins per segment
#define M               180         // number of segments
#define D_MAX           0.1f        // maximum distance from one point to a line
#define GRAD_MAX        0.17633f    // maximum gradient of the line = 10 degree
#define GRAD_MIN        0.01

namespace GPB{

struct line{
    double ls;
    size_t start;
    size_t stop;
};

}

class GPB_Segmentation : public Segmentation{
private:
    // generell parameters
    double Rmax;         // maximum distance considered
    double B;            // max radius initial seed
    double Ts;           // max height initial seed
    double a_res;        // angular size per segment
    double r_res;        // size per bin

    // hyperparameters
    double a, sn2, sf2, Tr, Tdata, Tmodel;

    // variables
    // array containing sqrt(x²+y²) for each point in PCL
    std::vector<double> calc_R;
    // matrix containing all points within a bin within a segment
    boost::array<boost::array<pcl::PointIndices, N > , M > PG;
    // matrix containing the minimum point of each bin within each segment
    boost::array<boost::array<pcl::PointIndices, N > , M > Pbmn;
    // gradients for each fitted line
    std::vector<double > scale_length;
    void polarGridMap();
    void fitline(size_t segment);
    std::vector<size_t> seed(size_t segment);
    cv::Mat gpr_model(size_t segment, std::vector<size_t> sp);
    std::vector<size_t> extract(size_t segment, std::vector<size_t> old);
    std::vector<size_t> gpr_eval(cv::Mat modelInv, std::vector<size_t> test, size_t segment, std::vector<size_t> sp);

public:

    GPB_Segmentation(int maxClusterSize = 2500, int minClusterSize = 25, double clusterTolerance = 0.30);
    virtual ~GPB_Segmentation();
    void removeFloorGPB(pcl::PointCloud<pcl::PointXYZ>::Ptr out);



};


#endif /* GPB_SEGMENTATION_H_ */

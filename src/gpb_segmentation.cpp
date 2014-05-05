/*
 * GPB_Segmentation.cpp
 *
 *  Created on: 24.04.2014
 *      Author: Michael
 */
#include <ros/ros.h>
#include "gpb_segmentation.hpp"
#include <boost/foreach.hpp>
#include <stdio.h>

GPB_Segmentation::GPB_Segmentation(int maxClusterSize, int minClusterSize, double clusterTolerance) :
    Segmentation(maxClusterSize, minClusterSize, clusterTolerance){

    Rmax = 20;
    a_res = (double) 2 * M_PI / M;
    r_res = (double) Rmax / 100;

    B = 10;
    Ts = 0.30;
    a = 6.2978;
    sf2 = 0.0528;
    sn2 = 0.0012;
    Tdata = 3;
    Tmodel = 0.04;
    Tr = 0.30;

}

GPB_Segmentation::~GPB_Segmentation() {

}

void GPB_Segmentation::removeFloorGPB(pcl::PointCloud<pcl::PointXYZ>::Ptr out){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    polarGridMap();
    pcl::PointIndices ground;
    for (size_t i = 0; i < M; i++) {
        //        ROS_INFO("Loop %d", i);
        // fitline
        fitline(i);

        if(scale_length.empty())
            continue;

        // estimate seeds -> sp isa vector containing the indices from PG which form the current seed
        std::vector<size_t> sp;
        std::vector<size_t> snew = seed(i);

        cv::Mat modelInv;
        // GP regression and seed evaluation
        //        ROS_INFO("While");
        while(!snew.empty()){
//                        ROS_INFO("seed %d", snew.size());
            sp.insert(sp.end(), snew.begin(), snew.end());
            cv::Mat model = gpr_model(i, sp);
//                        ROS_INFO("model");
            cv::invert(model, modelInv);
            //            ROS_INFO("inv");
            std::vector<size_t> test = extract(i, sp);
            //            ROS_INFO("extract");
            snew = gpr_eval(modelInv, test, i, sp);
            //            ROS_INFO("eval");

        }
        //        ROS_INFO("sp %d", sp.size());
        // pointwise segementation
        if(!sp.empty()){
            size_t cnt = 0;
            double l, l2, lh, lh2, rh, k;

            cv::Mat heights(sp.size(), 1, cv::DataType<double>::type);
            for(size_t j = 0; j < sp.size(); j++){
                size_t idx = PG.at(i).at(sp.at(j)).indices.at(0);
                heights.at<double>(j,0) = cloud->points.at(idx).z + 0.77;
            }



            for (size_t j = 0; j < N; j++) {
                if(!Pbmn.at(i).at(j).indices.empty()){
//                                       ROS_INFO("In here scale_length size = %d", scale_length.size());
                    double r = (2*j-1) * r_res / 2;
                    cv::Mat krsp(1, sp.size(), cv::DataType<double>::type);
                    l = scale_length.at(cnt);
                    l2 = pow(l, 2);

                    for(size_t h = 0; h < sp.size(); h++) {
                        size_t hdx = PG.at(i).at(sp.at(h)).indices.at(0);
                        lh = scale_length.at(sp.at(h));
                        lh2 = pow(lh, 2);
                        rh = calc_R.at(hdx);

                        k = sf2 * sqrt(l) * sqrt(lh) * sqrt((l2 + lh2)/2) * exp(-(2*pow(r - rh,2))/(l2 + lh2));

                        krsp.at<double>(0,h) = k;
                    }
                    //                    ROS_INFO("krsp %d x %d, modelInv %d x %d, height %d x %d", krsp.rows, krsp.cols, modelInv.rows, modelInv.cols, heights.rows, heights.cols);
                    cv::Mat zpredictMat = krsp * modelInv * heights;

                    double zpredict = 0;
                    if(zpredictMat.rows == 1 && zpredictMat.cols == 1){
                        zpredict = zpredictMat.at<double>(0,0);
                    }

                    for(size_t f = 0; f < Pbmn.at(i).at(j).indices.size(); f++){
                        if(!Pbmn.at(i).at(j).indices.empty()){
                            if((cloud->points.at(Pbmn.at(i).at(j).indices.at(f)).z - zpredict + 0.77) <= Tr){
                                ground.indices.push_back(Pbmn.at(i).at(j).indices.at(f));
                            }
                        }
                    }
                    cnt++;
                }
            }


        }
        scale_length.clear();
    }

    calc_R.clear();
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (boost::make_shared<pcl::PointIndices>(ground));

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud = *cloud_f;
    *out = *cloud;

}

void GPB_Segmentation::polarGridMap(){
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            PG.at(i).at(j) = pcl::PointIndices();
            Pbmn.at(i).at(j) = pcl::PointIndices();
        }
    }
    for (size_t i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZ *point = &(cloud->points.at(i));
        double r = sqrt( pow( point->x, 2) + pow( point->y, 2)); // r = sqrt(x² + y²)
        calc_R.push_back(r);                                    // store all r to save calculation time
        if(r < Rmax){
            double a = atan2(point->y, point->x);                // alpha = atan(y,x)
            if (a < 0)
                a += 2 * M_PI;

            // Fill segments
            int segment = (int) (a / a_res);
            int bin = (int) (r / r_res);
            Pbmn.at(segment).at(bin).indices.push_back(i);

            // Get minima
            if (PG.at(segment).at(bin).indices.size() == 0){
                PG.at(segment).at(bin).indices.push_back(i);
            } else if (PG.at(segment).at(bin).indices.at(0) > point->z) {
                PG.at(segment).at(bin).indices.at(0) = i;
            }
        }
    }
}

void GPB_Segmentation::fitline(size_t segment) {
    std::vector< cv::Vec2f > points;
    cv::Vec4f line_params;         // (vx, vy, x0, y0)
    std::vector<GPB::line > lines;
    bool newLine = true;
    GPB::line line;

    for (size_t i = 0; i < PG.at(segment).size(); i++) {
        if (!PG.at(segment).at(i).indices.empty()){           // TODO: check necessary --> yes
            // construct vector points
            size_t idx = PG.at(segment).at(i).indices.at(0);
            points.push_back(cv::Vec2f(calc_R.at(idx), cloud->points.at(idx).z));

            if(points.size() >= 2){

                //                for (int j = 0; j < points.size(); ++j) {
                //                    ROS_INFO("r = %f; z = %f;",points.at(j)[0], points.at(j)[1]);
                //                }

                // fitLine to current list of points
                cv::fitLine(points, line_params, CV_DIST_L2, 0, 0.1, 0.1);
                // vector direction of the line
                cv::Vec2f n(line_params[0],line_params[1]);
                // point on the line
                cv::Vec2f k(line_params[2],line_params[3]);
                //                ROS_INFO("%f %f %f %f", line_params[0], line_params[1],line_params[2],line_params[3]);
                // d = ||(k-p)-((k-p)'*n)*n||
                // calculate distance for start and endpoint to line
                double dstart = cv::norm((k - points.front()) - ((k - points.front()).dot(n))*n);
                double dend  = cv::norm((k - points.back()) - ((k - points.back()).dot(n))*n);
                // get the maximum
                double dmax = (dstart > dend) ? dstart : dend;
                // calculate gradient
                double gradient = fabs(line_params[1]/line_params[0]);

                //                ROS_INFO("distance = %f, gradient = %f", dmax, atan(gradient) * 180 / M_PI);

                // check line condition:
                // distance from line to start or endpoint should be smaller than D_MAX
                // gradient of the line should be smaller than GRAD_MAX
                if(dmax <= D_MAX && gradient <= GRAD_MAX){
                    if(newLine){
                        newLine = false;
                        if (gradient < GRAD_MIN){
                            line.ls = a * log(1/gradient);
                        } else {
                            line.ls = a * log(1/GRAD_MIN);
                        }
                        line.stop = i;
                        lines.push_back(line);
                    } else {
                        if (gradient < GRAD_MIN){
                            lines.at(lines.size() - 1).ls = a * log(1/gradient);
                        } else {
                            lines.at(lines.size() - 1).ls = a * log(1/GRAD_MIN);
                        }
                        lines.at(lines.size() - 1).stop = i;
                        /*                        ROS_INFO("last line gradient %f", atan(lines.at(lines.size() - 1).gradient) * 180 / M_PI)*/;
                    }
                } else {
                    points.clear();
                    newLine = true;
                }
            } else if (points.size() == 1){
                line.start = i;
            }


        }
    }

//    ROS_INFO("lines.size() = %d", lines.size());
    // fill length-scale vector
    if(lines.size() > 0) {
        int last = -1;
        int current = 0;
        int i = 0;
        while (i < PG.at(segment).size()){
//            ROS_INFO("%d %d %d", current, last, lines.size());
            double ls = 0;
            if (!PG.at(segment).at(i).indices.empty()){
                if (current >= lines.size()){
//                    ROS_INFO("I'm here");
                    ls = (lines.at(last).stop + 1 - i) / lines.at(last).stop * lines.at(last).ls + lines.at(last).ls;
//                    ls = lines.at(last).ls;
                    scale_length.push_back(ls);
                } else {
                    if (i < lines.at(current).start){
                        if (last == -1){
                            ls = i / lines.at(current).start * lines.at(current).ls;
//                            ls = lines.at(current).ls;
                            scale_length.push_back(ls);
                        } else {
                            ls = lines.at(current).ls + (lines.at(current).start - i)
                                    / (lines.at(current).start - lines.at(last).stop)
                                    * (lines.at(last).ls - lines.at(current).ls);
                            scale_length.push_back(ls);
                        }
                    } else if (i >= lines.at(current).start && i <= lines.at(current).stop){
                        ls = lines.at(current).ls;
                        scale_length.push_back(ls);
                    } else if (i > lines.at(current).stop) {
                        current++;
                        last++;
                        i--;
                    }
                }

            } else {
                scale_length.push_back(0);
            }
            i++;
        }
    }



}

std::vector<size_t> GPB_Segmentation::seed(size_t segment){
    // Calculate new seeds
    std::vector<size_t> snew;
    for (int i = 0; i < PG.at(segment).size(); i++) {
        if (!PG.at(segment).at(i).indices.empty()){
            size_t idx = PG.at(segment).at(i).indices.at(0);
            //            ROS_INFO("height %f", cloud->points.at(idx).z);
            if(calc_R.at(idx) <= B && cloud->points.at(idx).z <= Ts){
                snew.push_back(i);
            }
        }
    }
    return snew;
}

cv::Mat GPB_Segmentation::gpr_model(size_t segment, std::vector<size_t> sp){
    size_t size = sp.size();
    //    ROS_INFO("size model: %d x %d", size, size);
    //    ROS_INFO("length_scale.size() = %d", scale_length.size());
    //    for(int i = 0; i < scale_length.size(); i++){
    //        std::cout << scale_length.at(i) << " ";
    //    }
    //    std::cout << std::endl;

    //    ROS_INFO("sp:");
    //    for(int i = 0; i < sp.size(); i++){
    //        std::cout << sp.at(i) << " ";
    //    }
    //    std::cout << std::endl;

    cv::Mat model(size, size, cv::DataType<double>::type);
    double li, lj, li2, lj2, ri, rj, k;
    for (size_t i = 0; i < size; i++) {
        size_t idx = PG.at(segment).at(sp.at(i)).indices.at(0);
        li = scale_length.at(sp.at(i));
        li2 = pow(li, 2);
        ri = calc_R.at(idx);
        for (size_t j = 0; j < size; j++) {
            size_t jdx = PG.at(segment).at(sp.at(j)).indices.at(0);
            lj = scale_length.at(sp.at(j));
            lj2 = pow(lj, 2);
            rj = calc_R.at(jdx);

            k = sf2 * sqrt(li) * sqrt(lj) * sqrt((li2 + lj2)/2) * exp(-(2*pow(ri - rj,2))/(li2 + lj2));

            if(i == j)
                k += sn2;

            model.at<double>(i,j)=k;
        }
    }
    return model;
}

std::vector<size_t> GPB_Segmentation::extract(size_t segment, std::vector<size_t> sold){
    // Extract used seeds from segment
    std::vector<size_t> ext;
    for (int i = 0; i < PG.at(segment).size(); i++) {
        if(!PG.at(segment).at(i).indices.empty()){
            bool keep = true;
            for (int j = 0; j < sold.size(); ++j) {
                if(i == sold.at(j)){
                    keep = false;
                }
            }
            if(keep){
                ext.push_back(i);
            }
        }
    }
    return ext;
}

std::vector<size_t>  GPB_Segmentation::gpr_eval(cv::Mat modelInv, std::vector<size_t> test, size_t segment, std::vector<size_t> sp) {

    std::vector<size_t> snew;
    cv::Mat krsp(1, sp.size(), cv::DataType<double>::type);

    cv::Mat heights(sp.size(), 1, cv::DataType<double>::type);
    for(size_t i = 0; i < sp.size(); i++){
        size_t idx = PG.at(segment).at(sp.at(i)).indices.at(0);
        heights.at<double>(i,0) = cloud->points.at(idx).z;
    }


    double li, lj, li2, lj2, ri, rj, k;
    double krr;
    for (size_t i = 0; i < test.size(); i++) {

        size_t idx = PG.at(segment).at(test.at(i)).indices.at(0);
        li = scale_length.at(test.at(i));
        li2 = pow(li, 2);
        ri = calc_R.at(idx);
        for(size_t j = 0; j < sp.size(); j++) {
            size_t jdx = PG.at(segment).at(sp.at(j)).indices.at(0);
            lj = scale_length.at(sp.at(j));
            lj2 = pow(lj, 2);
            rj = calc_R.at(jdx);

            k = sf2 * sqrt(li) * sqrt(lj) * sqrt((li2 + lj2)/2) * exp(-(2*pow(ri - rj,2))/(li2 + lj2));

            krsp.at<double>(0,j)=k;
        }
        krr = sf2 * sqrt(li) * sqrt(li) * sqrt((li2 + li2)/2) * exp(-(2*pow(ri - ri,2))/(li2 + li2)) + sn2;

        cv::Mat krspTrans;
        cv::transpose(krsp, krspTrans);

        cv::Mat zmeanMat = krsp * modelInv * heights;
        double zmean = 0;
        if(zmeanMat.rows == 1 && zmeanMat.cols == 1){
            zmean = zmeanMat.at<double>(0,0);
        }

        cv::Mat tmpMat = krsp * modelInv * krspTrans;
        double tmp = 0;
        if(tmpMat.rows == 1 && tmpMat.cols == 1){
            tmp = tmpMat.at<double>(0,0);
        }

        double Vz = fabs(krr - tmp);

        double ttmp = fabs(cloud->points.at(idx).z - zmean)/sqrt(sn2 + Vz);
        if (Vz < Tmodel && ttmp <= Tdata){
            snew.push_back(test.at(i));
        }
    }
    return snew;
}





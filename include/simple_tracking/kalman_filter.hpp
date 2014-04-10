/*
 * KF.h
 *
 *  Created on: 24.03.2014
 *      Author: Michael
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Eigen/Dense>

class KalmanFilter {
private:
    Eigen::Matrix4f F;
    Eigen::MatrixXf H;
    Eigen::Matrix4f Q;
    Eigen::Matrix2f R;
    Eigen::Matrix4f Ppri;
    Eigen::Vector4f Xpri;
    Eigen::Matrix2f S;
    Eigen::MatrixXf W;
    Eigen::Vector2f Ypri;
    Eigen::Vector2f v;
    Eigen::Vector4f Xpos;
    Eigen::Matrix4f Ppos;

public:
    KalmanFilter(float dt, float q, float r, float P, float x0, float y0);
    virtual ~KalmanFilter();

    void setPosition(float x0, float y0);

    void covUpdate();
    void stateUpdate(float xN, float yN);
    void prediction();
    void process(float xN, float yN);

    void setTimeStep(float dt);
    Eigen::Vector4f getState();

};

#endif /* KALMAN_FILTER_H_ */

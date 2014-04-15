/*
 * KalmanFilter.cpp
 *
 *  Created on: 24.03.2014
 *      Author: Michael
 */

#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(float dt, float q, float r, float P, float x0 = 0, float y0 = 0) {

    F << 1,0,dt,0,
         0,1,0,dt,
         0,0,1,0,
         0,0,0,1;

    H = Eigen::MatrixXf(2,4);
    H << 1,0,0,0,
         0,1,0,0;

    Q << q,0,0,0,
         0,q,0,0,
         0,0,q,0,
         0,0,0,q;

    R << q,0,
         0,q;

    Ppri << P,0,0,0,
            0,P,0,0,
            0,0,P,0,
            0,0,0,P;

    Xpri << x0,y0,0,0;

    W = Eigen::MatrixXf(4,2);
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::setPosition(float x0, float y0){
    Xpri << x0,y0,0,0;
}

void KalmanFilter::covUpdate(){
    S = R + H * Ppri * H.transpose();
    W = Ppri * H.transpose() * S.inverse();
    Ppos = Ppri - W * S * W.transpose();
}

void KalmanFilter::stateUpdate(float xN, float yN){
    Ypri = H * Xpri;
    Eigen::Vector2f Y;
    Y << xN, yN;
    v = Y - Ypri;
    Xpos = Xpri + W * v;
}

void KalmanFilter::prediction(){
    Xpri = F * Xpos;
    Ppri = F * Ppos * F.transpose() + Q;
}

void KalmanFilter::process(float xN, float yN){
    covUpdate();
    stateUpdate(xN, yN);
    prediction();
}

void KalmanFilter::setTimeStep(float dt){
    F << 1,0,dt,0,
         0,1,0,dt,
         0,0,1,0,
         0,0,0,1;
}

Eigen::Vector4f KalmanFilter::getState(){
    return Xpos;
}

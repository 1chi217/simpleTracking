/*
 * KalmanFilter.cpp
 *
 *  Created on: 31.03.2014
 *      Author: Michael
 */

#include "path.hpp"
#include <ros/ros.h>
Path::Path() {
    firstrun = true;
}

Path::~Path() {
    // TODO Auto-generated destructor stub
}

void Path::update(double dx, double dy, ros::Time now){
    if(firstrun){
        firstrun = false;
        angular = 0;
        linear = 0;
    } else {
        double heading = atan2(dy, dx);
        double timeStep = now.toSec() - last.toSec();

        double distance = sqrt(pow(dx,2) + pow(dy,2));
        if(distance < 4.5 || fabs(heading) > M_PI_4){
            linear = 0;
            angular = 0;
        } else {
            linear = timeStep*distance; //sqrt(pow(dx - oldX,2) + pow(dy - oldY,2)) / timeStep;
            angular = heading; // / timeStep;
        }
        last = now;
        oldX = dx;
        oldY = dy;
//         ROS_INFO("heading %f timestep %f", heading, timeStep);
    }


}

ackermann_msgs::AckermannDrive Path::getCmdCar() {
    ackermann_msgs::AckermannDrive ack_cmd;
    ack_cmd.steering_angle = 0;
    ack_cmd.speed = 0;
    return ack_cmd;
}

geometry_msgs::Twist Path::getCmdDiff(){
    geometry_msgs::Twist vel_cmd;
    vel_cmd.angular.z = angular;
    vel_cmd.linear.x = linear;
    return vel_cmd;
}


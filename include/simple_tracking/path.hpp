/*
 * path.hpp
 *
 *  Created on: 24.03.2014
 *      Author: Michael
 */

#ifndef PATH_H_
#define PATH_H_

#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

class Path {
private:
    float angular, linear;
    float oldX, oldY;
    ros::Time last;
    bool firstrun;

public:
    Path();
    virtual ~Path();

    // (dx, dy) position of human relative to robot, e.g. velodyne or base_link frame
    void update(double dx, double dy, ros::Time now);

    ackermann_msgs::AckermannDrive getCmdCar();
    geometry_msgs::Twist getCmdDiff();

};

#endif /* PATH_H_ */

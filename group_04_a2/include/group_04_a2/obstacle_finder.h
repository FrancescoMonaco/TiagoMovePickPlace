#ifndef OBSTACLE_FINDER_H
#define OBSTACLE_FINDER_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

std::vector<geometry_msgs::PoseStamped> obstacle_finder_function(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

#endif
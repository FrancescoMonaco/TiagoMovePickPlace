#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <group_04_a1/tiago_goal.h>


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

bool hasObstacle();

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg);

void motion (geometry_msgs::PoseStamped pos);

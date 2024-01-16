#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <group_04_a2/tiago_goal.h>

class TiagoMotionController {
public:
    TiagoMotionController() : nh_(), goal_reached_(true) {
        //Find your pose not in odom but in robot_pose
        //Odom doesnt exist as a node here
        odom_sub_ = nh2_.subscribe("robot_pose", 1, &TiagoMotionController::odomCallback, this);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);
        goal_sub_ = nh_.subscribe("tiago_goal", 1, &TiagoMotionController::goalCallback, this);
        laser_sub_ = nh3_.subscribe("scan", 1, &TiagoMotionController::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
        // Store the laser scan data for later use
        laser_scan = *laser_msg;
    }

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg) {
    ROS_INFO("odomCallback");
    if (!goal_reached_) {
        // Compute velocities to move toward the goal based on odometry data
        geometry_msgs::Twist vel_cmd;
        
        // Extract current robot pose from odometry message
        geometry_msgs::Pose current_pose = odom_msg->pose.pose;
        ROS_INFO("Current pose: (%f, %f)", current_pose.position.x, current_pose.position.y);

        // Check if there is an obstacle in front of the robot
        if (hasObstacle()) {
            // If there is an obstacle, rotate the robot
            vel_cmd.linear.x = 0.0;
            vel_cmd.angular.z = 5.2;  // Adjust angular velocity as needed for rotation
        } else {
            // If there is no obstacle, move the robot forward
            vel_cmd.linear.x = 5.2;  // Adjust linear velocity as needed for forward movement
            vel_cmd.angular.z = 0.0;
        }

        // Publish computed velocities
        vel_pub_.publish(vel_cmd);
        ROS_INFO("Moving toward goal");
        // Check if the goal is reached (for example, based on a distance threshold)
        if (isGoalReached(current_pose, goal_)) {
            goal_reached_ = true;
            // Stop the robot when the goal is reached
            vel_cmd.linear.x = 0.0;
            vel_cmd.angular.z = 0.0;
            vel_pub_.publish(vel_cmd);
        }

        //Wait for 1 second
        ros::Duration(1.0).sleep();
    }
}

    void goalCallback(const geometry_msgs::Pose goal_msg) {
        // Store the goal pose for later use
        goal_ = goal_msg;
        // Set the goal reached flag to false
        goal_reached_ = false;
        ROS_INFO("Goal received");
    }

    bool isGoalReached(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& goal_pose) {
        // Compute the distance between the current robot pose and the goal pose
        double distance = sqrt(pow(current_pose.position.x - goal_pose.position.x, 2) +
                               pow(current_pose.position.y - goal_pose.position.y, 2));

        // Dist thresh
        double distance_threshold = 0.1; 

        // Check if the goal is reached
        if (distance < distance_threshold) {
            return true;
        }

        return false;
    }

    bool hasObstacle() {
    // Check if laser scan data is available
    if (laser_scan.ranges.empty()) {
        // No laser scan data available, consider it as an obstacle
        return true;
    }

    // Define the range index corresponding to the front of the robot
    // You may need to adjust this based on your robot's configuration
    int front_index = laser_scan.ranges.size() / 2;

    // Define a distance threshold for considering obstacles
    double obstacle_threshold = 1.5;  // Adjust as needed

    // Check if there's an obstacle in front of the robot based on the range value
    if (laser_scan.ranges[front_index] < obstacle_threshold) {
        // Obstacle detected
        return true;
    }

    // No obstacle detected
    return false;
}

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh2_;
    ros::NodeHandle nh3_;
    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber laser_sub_;
    bool goal_reached_ = false;
    geometry_msgs::Pose goal_;
    sensor_msgs::LaserScan laser_scan;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tiago_motion_controller");
    TiagoMotionController motion_controller;
    ros::spin();
    return 0;
}


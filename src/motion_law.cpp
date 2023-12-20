#include <group_04_a1/motion_law.h>

sensor_msgs::LaserScan laser_scan;
bool goal_reached_ = false;
bool pass = false;
ros::Publisher vel_pub_;
ros::Subscriber laser_sub_;
ros::Subscriber odom_sub_;

/**
*  Read the data form LaserScan and store them in laser_scan
*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
        // Store the laser scan data for later use
        laser_scan = *laser_msg;
    }

/**
* Check if there is an obstacle in front of the robot
*/
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
    double obstacle_threshold = 3;

    // Check if there's an obstacle in front of the robot based on the range value
    if (laser_scan.ranges[front_index] < obstacle_threshold) {
        // Obstacle detected
		goal_reached_ = true;
        return true;
    }

    // No obstacle detected
    return false;
}
/**
* Function that sends the commands to move the robot
*/
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg) {
    if (!goal_reached_) {
        // Compute velocities to move toward the goal based on odometry data
        geometry_msgs::Twist vel_cmd;
        
        // Extract current robot pose from odometry message
        geometry_msgs::Pose current_pose = odom_msg->pose.pose;

        // Check if there is an obstacle in front of the robot
        if (hasObstacle()) {
            // If there is an obstacle, rotate the robot
            vel_cmd.linear.x = -0.5;
            vel_cmd.angular.z = 0.3;  // Adjust angular velocity as needed for rotatio
			pass = true;
            
        } else {
            // If there is no obstacle, move the robot forward
            vel_cmd.linear.x = 1;  // Adjust linear velocity as needed for forward movement
            vel_cmd.angular.z = 0.0;
        }

        // Publish computed velocities
        vel_pub_.publish(vel_cmd);

        // Check if the goal is reached (based on a distance threshold)
       /* if (isGoalReached(current_pose, goal_)) {
            goal_reached_ = true;
            // Stop the robot when the goal is reached
            vel_cmd.linear.x = 0.0;
            vel_cmd.angular.z = 0.0;
            vel_pub_.publish(vel_cmd);
        }*/
    }
}

/**
* Create the nodes and subscribe to the specific topics
*/
void motion (geometry_msgs::PoseStamped pos){
	ros::NodeHandle nh_, nh1_, nh2_;
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);
	laser_sub_ = nh1_.subscribe("scan", 1, laserCallback);
    odom_sub_ = nh2_.subscribe("robot_pose", 1, odomCallback);
    // repeat until the robot passes the narrow corridor 
	while (!pass){
        ros::spinOnce();
	}
    // close all the nodes allowing the movement of the robot using move_base
	vel_pub_.shutdown();
	laser_sub_.shutdown();
	odom_sub_.shutdown();
	return;
	}

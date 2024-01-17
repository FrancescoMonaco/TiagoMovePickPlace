#ifndef CLIENT_H
#define CLIENT_H

//*** Includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <group_04_a2/TiagoAction.h>
#include <group_04_a2/human_client.h>
#include <group_04_a2/camera.h>

//*** Movement functions declaration

/// @brief Function that is called when the goal completes
/// @param state  
/// @param result 
void doneCb(const actionlib::SimpleClientGoalState& state,
            const group_04_a2::TiagoResultConstPtr& result);

/// @brief Function that is called when the goal becomes active
void activeCb();

/// @brief Function that is called when feedback is received
/// @param feedback pointer to the feedback section of the action
void feedbackCb(const group_04_a2::TiagoFeedbackConstPtr& feedback);

/// @brief Function that creates a goal message, if the goal is not valid it ends with an error message
/// @param x x coordinate of the goal
/// @param y y coordinate of the goal
/// @param z z coordinate of the goal
/// @param t1 roll of the goal
/// @param t2 pitch of the goal
/// @param t3 yaw of the goal
/// @param linear_init if true the robot will move in the corridor
group_04_a2::TiagoGoal createGoal(double x, double y, double z, double t1, double t2, double t3, bool linear_init=false);

/// @brief Returns a map with waypoints to move around the table
/// @return std::map<int, geometry_msgs::PoseStamped>, the key is the number of the waypoint
std::map<int, geometry_msgs::PoseStamped> getPositionMap();

//*** Camera functions declaration

/// @brief Contact the camera server and return the result of the detection
/// @param color_recognition if true the camera will detect the color of the objects otherwise will use the AprilTags
/// @return Pointer to the result of CameraAction
group_04_a2::CameraResultConstPtr cameraDetection(bool color_recognition=false);

/// @brief Function that is called when the goal of the camera completes
/// @param state 
/// @param result Result of the camera action
void doneCbCamera(const actionlib::SimpleClientGoalState &state, const group_04_a2::CameraResultConstPtr &result);

/// @brief Function that is called when the goal of the camera becomes active
void activeCbCamera();

/// @brief Function that is called when feedback of the camera is received
void feedbackCbCamera(const group_04_a2::CameraFeedbackConstPtr &feedback);

//*** Arm functions declaration

#endif
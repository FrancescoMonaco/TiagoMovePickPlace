#ifndef CLIENT_H
#define CLIENT_H

//*** Includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <group_04_a1/TiagoAction.h>

//*** Callback functions declaration

/// @brief Function that is called when the goal completes
/// @param state  
/// @param result 
void doneCb(const actionlib::SimpleClientGoalState& state,
            const group_04_a1::TiagoResultConstPtr& result);

/// @brief Function that is called when the goal becomes active
void activeCb();

/// @brief Function that is called when feedback is received
/// @param feedback pointer to the feedback section of the action
void feedbackCb(const group_04_a1::TiagoFeedbackConstPtr& feedback);

/// @brief Function that creates a goal message, if the goal is not valid it ends with an error message
/// @param x x coordinate of the goal
/// @param y y coordinate of the goal
/// @param z z coordinate of the goal
/// @param t1 roll of the goal
/// @param t2 pitch of the goal
/// @param t3 yaw of the goal
group_04_a1::TiagoGoal createGoal(double x, double y, double z, double t1, double t2, double t3);

#endif
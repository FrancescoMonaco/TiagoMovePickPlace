#include <group_04_a1/server.h>

//*** Function implementation

    void Tiago::goalCB(){
        goal_ = as_.acceptNewGoal()->goal_pose;
        // Send the goal to move_base_simple/goal
        pub_.publish(goal_);
        // Send feedback to the client
         // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
         // Set the feedback message
        feedback_.feedback_message = "Goal sent";
        as_.publishFeedback(feedback_);
    }

    void Tiago::preemptCB(){
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }

    void Tiago::poseCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
        // Check if the robot has reached the goal in /move_base/result
        if(msg->status.status == 3){
            success_ = true;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot has reached the goal";
            // Set the status
            feedback_.status = msg->status.status;
            as_.publishFeedback(feedback_);
            // Call the object position function
            computeObjectPosition();
        }
            // If it failed becuase the point is outside the map
        else if (msg->status.status == 4 && 
                (msg->status.text.find("oscillating") == std::string::npos)){
            success_ = false;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot has not reached the goal: " + msg->status.text;
            // Set the status
            feedback_.status = msg->status.status;
            as_.publishFeedback(feedback_);
        }
            // Else it failed because of an obstacle
            // => Motion Control Law takes control

    }

    void Tiago::feedCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
        if(!as_.isActive())
            return; 
        
        if (success_){
        }
        else{
            // Compare the actual position with the previous one
            // If the truncation of the actual position is equal to the truncation of the previous one
            // => The robot is not moving
            if((int)(pose_actual_.pose.position.x*1000) == (int)(pose_previous_.pose.position.x*1000) &&
                (int)(pose_actual_.pose.position.y*1000) == (int)(pose_previous_.pose.position.y*1000)){
                // Return feedback to the client
                // Set the feedback header
                feedback_.head_feedback.seq++;
                feedback_.head_feedback.stamp = ros::Time::now();
                feedback_.head_feedback.frame_id = "Goal feedback";
                // Set the feedback message
                feedback_.feedback_message = "The robot is stopped";
                as_.publishFeedback(feedback_);
            }
            else{
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot is moving to the goal";
            as_.publishFeedback(feedback_);
            }
        }
    }

    void Tiago::updateCB(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
        // Save the old position
        pose_previous_= pose_actual_;
        pose_actual_ = msg->feedback.base_position;
    }

    void Tiago::computeObjectPosition(){

        as_.setSucceeded(result_);
        return;
    }
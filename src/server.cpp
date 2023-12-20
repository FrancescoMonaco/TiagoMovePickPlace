#include <group_04_a1/server.h>
#include <group_04_a1/obstacle_finder.h>
#include <group_04_a1/motion_law.h>
//*** Function implementation

    void Tiago::goalCB(){
        goal_ = as_.acceptNewGoal()->goal_pose;
        //Wait some seconds for the arm to tuck
        // Send feedback of tucking
        // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
        // Set the feedback message
        feedback_.feedback_message = "Tucking the arm and activating ML";
        as_.publishFeedback(feedback_);
        ros::Duration(13.0).sleep();
        // Motion Law
        motion(goal_);
        // Send the goal to move_base_simple/goal
        pub_.publish(goal_);
        // Send feedback to the client
         // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
         // Set the feedback message
        feedback_.feedback_message = "Goal sent to move_base";
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
        else if (msg->status.status == 4 &&         // Else it failed because of an obstacle
                                                    // => Motion Control Law takes control
                (msg->status.text.find("oscillating") != std::string::npos)){
            success_ = false;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "Narrow passage, switching to Motion Control Law";
            // Set the status
            feedback_.status = 0;
            as_.publishFeedback(feedback_);

            motion(goal_);
        }


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

    void Tiago::laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        laser_msg_ = msg;
    }

    void Tiago::computeObjectPosition(){
        // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "Computing the position of the objects";
            as_.publishFeedback(feedback_);

        // Call the object position function, it wants as input sensor_msgs::LaserScan::ConstPtr&
        std::vector<geometry_msgs::PoseStamped> obstacle_poses = obstacle_finder_function(laser_msg_);
        // Return the result to the client
        result_.result_points = obstacle_poses;
        as_.setSucceeded(result_);
        return;
    }
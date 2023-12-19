#include <group_04_a1/client.h>

//*** Callback functions definition
void doneCb(const actionlib::SimpleClientGoalState& state,
            const group_04_a1::TiagoResultConstPtr& result)
{
    // Take the list of the poses from the result
    std::vector<double> pose_x;
    std::vector<double> pose_y;
    std::vector<double> pose_z;
    for(int i = 0; i < result->result_points.size(); i++)
    {
        pose_x.push_back(result->result_points[i].pose.position.x);
        pose_y.push_back(result->result_points[i].pose.position.y);
        //pose_z.push_back(result->result_points[i].pose.position.z);
    }
    // Print the list of the poses
    for(int i = 0; i < pose_x.size(); i++)
    {
        ROS_INFO("Object %d in position: (%f, %f)", i+1, pose_x[i], pose_y[i]);//, pose_z[i]);
    }
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal just went active");
}

void feedbackCb(const group_04_a1::TiagoFeedbackConstPtr& feedback)
{   
    // Take the string from the feedback and print it
    std::string feedback_string = feedback->feedback_message;
    ROS_INFO("Feedback: %s", feedback_string.c_str());
    if(feedback->status == 4) // If failed shutdown the node
    {
        ros::shutdown();
    }
}

group_04_a1::TiagoGoal createGoal(double x, double y, double z, double t1, double t2, double t3)
{   
    // Check if the goal is valid
    if(z != 0 || t3 != 0)
    {
        // Print an error message
        ROS_INFO("The goal is not valid, the robot can only move in the x-y plane");
        // Shutdown the node
        ros::shutdown();
    }

    // Check if the goal is in the map (TO DO)

    group_04_a1::TiagoGoal goal;
    // use geometry_msgs::PoseStamped to set the goal pose
    goal.goal_pose.header.frame_id = "map";
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y;
    goal.goal_pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(t1, t2, t3);
    goal.goal_pose.pose.orientation.x = q.x();
    goal.goal_pose.pose.orientation.y = q.y();
    goal.goal_pose.pose.orientation.z = q.z();
    goal.goal_pose.pose.orientation.w = q.w();
    // set the header of the goal
    goal.goal_pose.header.stamp = ros::Time::now();

    return goal;
}

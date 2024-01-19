#include <group_04_a2/client.h>

//*** Callbacks for the tiago server
void doneCb(const actionlib::SimpleClientGoalState& state,
            const group_04_a2::TiagoResultConstPtr& result)
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
}

void activeCb()
{
    ROS_INFO("Goal just went active");
}

void feedbackCb(const group_04_a2::TiagoFeedbackConstPtr& feedback)
{   
    // Take the string from the feedback and print it
    std::string feedback_string = feedback->feedback_message;
    ROS_INFO("Feedback: %s", feedback_string.c_str());
    if(feedback->status == 4) // If failed shutdown the node
    {
        ros::shutdown();
    }
}

//*** Callbacks for the camera server
void doneCbCamera(const actionlib::SimpleClientGoalState &state, const group_04_a2::CameraResultConstPtr &result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    // Print the result
    if (result->poses.size() == 0)
    {
        ROS_INFO("No objects detected");
    }
    else
    {
        for (int i = 0; i < result->ids.size(); i++)
        {
            ROS_INFO("Object %d detected at position (%f, %f, %f)", result->ids[i], result->poses[i].position.x, result->poses[i].position.y, result->poses[i].position.z);
        }
    }
}

void activeCbCamera()
{
    ROS_INFO("Goal just went active");
}

void feedbackCbCamera(const group_04_a2::CameraFeedbackConstPtr &feedback)
{
    return;
}

//*** Callbacks for the arm server
void arm_doneCb(const actionlib::SimpleClientGoalState& state, const group_04_a2::ArmResultConstPtr& result)
{
    ROS_INFO("Finished");
}

void arm_activeCb()
{
    ROS_INFO("Goal just went active");
}

void arm_feedbackCb(const group_04_a2::ArmFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of the Arm");
}

//*** Other functions
group_04_a2::TiagoGoal createGoal(double px, double py, double pz, double ox, double oy, double oz, double ow, bool linear_init)
{   
   /*// Check if the goal is valid
    if(z != 0 || t1 != 0 || t2 != 0)
    {
        // Print an error message
        ROS_INFO("The goal is not valid, the robot can only move in the x-y plane");
        // Shutdown the node
        ros::shutdown();
    }*/

    // Check if the goal is in the map (TO DO)

    group_04_a2::TiagoGoal goal;
    // use geometry_msgs::PoseStamped to set the goal pose
    goal.goal_pose.header.frame_id = "map";
    goal.goal_pose.pose.position.x = px;
    goal.goal_pose.pose.position.y = py;
    goal.goal_pose.pose.position.z = pz;
    goal.goal_pose.pose.orientation.x = ox;
    goal.goal_pose.pose.orientation.y = oy;
    goal.goal_pose.pose.orientation.z = oz;
    goal.goal_pose.pose.orientation.w = ow;
    goal.linear_init = linear_init;
    // set the header of the goal
    goal.goal_pose.header.stamp = ros::Time::now();

    return goal;
}

group_04_a2::CameraResultConstPtr cameraDetection(bool color_recognition){
    // Call the camera action server
    group_04_a2::CameraGoal goal;
    goal.color_recognition = color_recognition;
    // Create a client for the camera action server
    actionlib::SimpleActionClient<group_04_a2::CameraAction> ac_camera("tiago_camera", true);
    // Wait for the action server to start
    ROS_INFO("Waiting for camera action server to start.");
    ac_camera.waitForServer();
    ROS_INFO("Camera ction server started, sending goal.");
    // Send the goal to the action server
    ac_camera.sendGoal(goal, &doneCbCamera, &activeCbCamera, &feedbackCbCamera);
    // Wait for the result
    ac_camera.waitForResult();
    // Get the result
    group_04_a2::CameraResultConstPtr result = ac_camera.getResult();

    return result;
 } 

 group_04_a2::TiagoResultConstPtr move_to(geometry_msgs::PoseStamped pose_1, actionlib::SimpleActionClient<group_04_a2::TiagoAction> &ac, bool corridor){

    group_04_a2::TiagoGoal goal = createGoal(pose_1.pose.position.x, pose_1.pose.position.y, pose_1.pose.position.z, 
                                                pose_1.pose.orientation.x, pose_1.pose.orientation.y, 
                                                pose_1.pose.orientation.z, pose_1.pose.orientation.w, corridor);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();
    group_04_a2::TiagoResultConstPtr result = ac.getResult();
    return result;
 }

 group_04_a2::ArmResultConstPtr pick_place(std::vector<geometry_msgs::Pose> objects, std::vector<int> ids, bool pick){
    //Send pick goal to arm
    group_04_a2::ArmGoal arm_goal;
    arm_goal.poses = objects;
    arm_goal.ids = ids;
    arm_goal.pick = pick;

    // Initialize the arm client
    actionlib::SimpleActionClient<group_04_a2::ArmAction> arm_ac("tiago_arm", true);
    ROS_INFO("Waiting for arm server to start.");
    arm_ac.waitForServer();
    ROS_INFO("Arm server started, sending goal.");
    arm_ac.sendGoal(arm_goal, &arm_doneCb, &arm_activeCb, &arm_feedbackCb);

    // Wait for the end
    arm_ac.waitForResult();
    group_04_a2::ArmResultConstPtr result = arm_ac.getResult();
    return result;
 }

 std::map<int, geometry_msgs::PoseStamped> getPositionMap(){
    geometry_msgs::PoseStamped pose_1;
        // Table pose 1
    pose_1.header.stamp = ros::Time::now();
    pose_1.header.frame_id = "";
    pose_1.pose.position.x = 8.053;
    pose_1.pose.position.y = -1.982;
    pose_1.pose.position.z = 0;
    pose_1.pose.orientation.x = 0;
    pose_1.pose.orientation.y = 0;
    pose_1.pose.orientation.z = -1.5;

    geometry_msgs::PoseStamped pose_2;
        // Table pose 2
    pose_2.header.stamp = ros::Time::now();
    pose_2.header.frame_id = "";
    pose_2.pose.position.x = 7.473;
    pose_2.pose.position.y = -2.103;
    pose_2.pose.position.z = 0;
    pose_2.pose.orientation.x = 0;
    pose_2.pose.orientation.y = 0;
    pose_2.pose.orientation.z = -1.5;

    geometry_msgs::PoseStamped pose_3;
        // Pose 3
    pose_3.header.stamp = ros::Time::now();
    pose_3.header.frame_id = "";
    pose_3.pose.position.x = 7.883;
    pose_3.pose.position.y = -3.943;
    pose_3.pose.position.z = 0;
    pose_3.pose.orientation.x = 0;
    pose_3.pose.orientation.y = 0;
    pose_3.pose.orientation.z = 1.5;

    geometry_msgs::PoseStamped waypoint_1;
        // Waypoint 1
    waypoint_1.header.stamp = ros::Time::now();
    waypoint_1.header.frame_id = "";
    waypoint_1.pose.position.x = 8.86;
    waypoint_1.pose.position.y = -2.22;
    waypoint_1.pose.position.z = 0;
    waypoint_1.pose.orientation.x = 0;
    waypoint_1.pose.orientation.y = 0;
    waypoint_1.pose.orientation.z = -0.702;

    geometry_msgs::PoseStamped waypoint_2;
        // Waypoint 2
    waypoint_2.header.stamp = ros::Time::now();
    waypoint_2.header.frame_id = "";
    waypoint_2.pose.position.x = 9.35;
    waypoint_2.pose.position.y = -4.22;
    waypoint_2.pose.position.z = 0;
    waypoint_2.pose.orientation.x = 0;
    waypoint_2.pose.orientation.y = 0;

    geometry_msgs::PoseStamped waypoint_place;
        // Waypoint Place
    waypoint_place.header.stamp = ros::Time::now();
    waypoint_place.header.frame_id = "";
    waypoint_place.pose.position.x = 11.478;
    waypoint_place.pose.position.y = -2.317;
    waypoint_place.pose.position.z = 0;
    waypoint_place.pose.orientation.x = 0;
    waypoint_place.pose.orientation.y = 0;
    waypoint_place.pose.orientation.z = 1.5;
    waypoint_place.pose.orientation.w = 0.778;

    std::map<int, geometry_msgs::PoseStamped> position_map;
    position_map[1] = pose_1;
    position_map[2] = pose_2;
    position_map[3] = waypoint_1;
    position_map[4] = waypoint_2;
    position_map[5] = pose_3;
    position_map[6] = waypoint_place;
    return position_map;
}


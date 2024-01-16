#include <group_04_a2/client.h>
#include <group_04_a2/human_client.h>
#include <group_04_a2/camera.h>
#include <group_04_a2/arm.h>

// Define global variables
geometry_msgs::PoseStamped pose_1;
geometry_msgs::PoseStamped pose_2;
geometry_msgs::PoseStamped pose_3;
geometry_msgs::PoseStamped waypoint_1;
geometry_msgs::PoseStamped waypoint_place;

// Function to initialize poses
void InitializePoses();

void arm_doneCb(const actionlib::SimpleClientGoalState& state, const group_04_a2::ArmResultConstPtr& result);
void arm_activeCb();
void arm_feedbackCb(const group_04_a2::ArmFeedbackConstPtr& feedback);

//*** Main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "tiago_management");
    ROS_INFO("Starting tiago_management node");
    // Initialize the action client
    actionlib::SimpleActionClient<group_04_a2::TiagoAction> ac("tiago_pose", true);
    // Ask human_node for the order of the objects
    std::vector<int> objects = get_order();
    // Wait for the arm to tuck
    ros::Duration(13.0).sleep();
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    InitializePoses();
    // Go to a starting position
    group_04_a2::TiagoGoal goal = createGoal(8.5, 0, 0, 0, 0, 0, true);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();
    
    
    // Go to the first object
    
    goal = createGoal(pose_1.pose.position.x, pose_1.pose.position.y, pose_1.pose.position.z, pose_1.pose.orientation.x, pose_1.pose.orientation.y, pose_1.pose.orientation.z, pose_1.pose.orientation.w);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();
    
    
    // Go to the second object
    /*
    goal = createGoal(pose_2.pose.position.x, pose_2.pose.position.y, pose_2.pose.position.z, pose_2.pose.orientation.x, pose_2.pose.orientation.y, pose_2.pose.orientation.z, pose_2.pose.orientation.w);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();
    */
   
    // Get the camera results
    group_04_a2::CameraResultConstPtr camera_pointer = cameraDetection();

    //Send pick goal to arm
    group_04_a2::ArmGoal arm_goal;
    arm_goal.poses = camera_pointer->poses;
    arm_goal.ids = camera_pointer->ids;
    arm_goal.pick = true;

    // Initialize the arm client
    actionlib::SimpleActionClient<group_04_a2::ArmAction> arm_ac("tiago_arm", true);
    ROS_INFO("Waiting for arm server to start.");
    arm_ac.waitForServer();
    ROS_INFO("Arm server started, sending goal.");
    arm_ac.sendGoal(arm_goal, &arm_doneCb, &arm_activeCb, &arm_feedbackCb);

    // Wait for the end
    arm_ac.waitForResult();
    /*
   // Go to waypoint 1
    goal = createGoal(waypoint_1.pose.position.x, waypoint_1.pose.position.y, waypoint_1.pose.position.z, waypoint_1.pose.orientation.x, waypoint_1.pose.orientation.y, waypoint_1.pose.orientation.z, waypoint_1.pose.orientation.w);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();

    // Go to waypoint place
    goal = createGoal(waypoint_place.pose.position.x, waypoint_place.pose.position.y, waypoint_place.pose.position.z, waypoint_place.pose.orientation.x, waypoint_place.pose.orientation.y, waypoint_place.pose.orientation.z, waypoint_place.pose.orientation.w);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // Wait for the robot to reach the position
    ac.waitForResult();

    // Activate the camera to detect the color of the objects
    group_04_a2::CameraResultConstPtr camera_res = cameraDetection(true);
    //Print the ids
    for(int i = 0; i < camera_res->ids.size(); i++){
        ROS_INFO("id %d", camera_res->ids[i]);
    }
    */

    return 0;
}

//***IMPLEMENTATIONS

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






void InitializePoses(){
    // Pose 1
    pose_1.header.stamp = ros::Time::now();
    pose_1.header.frame_id = "";
    pose_1.pose.position.x = 8.053;
    pose_1.pose.position.y = -1.982;
    pose_1.pose.position.z = 0;
    pose_1.pose.orientation.x = 0;
    pose_1.pose.orientation.y = 0;
    pose_1.pose.orientation.z = -1.5;
    pose_1.pose.orientation.w = 0.896;

    // Pose 2
    pose_2.header.stamp = ros::Time::now();
    pose_2.header.frame_id = "";
    pose_2.pose.position.x = 7.473;
    pose_2.pose.position.y = -2.103;
    pose_2.pose.position.z = 0;
    pose_2.pose.orientation.x = 0;
    pose_2.pose.orientation.y = 0;
    pose_2.pose.orientation.z = -1.5;
    pose_2.pose.orientation.w = 0.896;

    // Pose 3
    pose_3.header.stamp = ros::Time::now();
    pose_3.header.frame_id = "";
    pose_3.pose.position.x = 7.883;
    pose_3.pose.position.y = -3.943;
    pose_3.pose.position.z = 0;
    pose_3.pose.orientation.x = 0;
    pose_3.pose.orientation.y = 0;
    pose_3.pose.orientation.z = 1.5;
    pose_3.pose.orientation.w = 0.669;

    // Waypoint 1
    waypoint_1.header.stamp = ros::Time::now();
    waypoint_1.header.frame_id = "";
    waypoint_1.pose.position.x = 8.86;
    waypoint_1.pose.position.y = -2.22;
    waypoint_1.pose.position.z = 0;
    waypoint_1.pose.orientation.x = 0;
    waypoint_1.pose.orientation.y = 0;
    waypoint_1.pose.orientation.z = -0.702;
    waypoint_1.pose.orientation.w = 0.7124;

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
}
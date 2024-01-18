#include <group_04_a2/client.h>

// Function to initialize poses
void InitializePoses();
geometry_msgs::PoseStamped pose_1;
geometry_msgs::PoseStamped pose_2;
geometry_msgs::PoseStamped pose_3;
geometry_msgs::PoseStamped waypoint_1;
geometry_msgs::PoseStamped waypoint_2;
geometry_msgs::PoseStamped waypoint_place;


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

    // Wait for the arm to tuck and go to a starting position
    ros::Duration(13.0).sleep();
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started");
    group_04_a2::TiagoGoal goal = createGoal(8.5, 0, 0, 0, 0, 0, true);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ac.waitForResult();

    // Substitute with the map one once the place works and the cycle is developed
    InitializePoses();
    
    // Go to the first object
    move_to(waypoint_2, ac);
    move_to(pose_3, ac);
    
    // Get the camera results
    group_04_a2::CameraResultConstPtr camera_pointer = cameraDetection();

    //Send pick goal to arm
    pick_place(camera_pointer->poses, camera_pointer->ids, true);
    
    // Go to waypoint 1
    //move_to(waypoint_1, ac);
    /*
    // Go to waypoint place
    group_04_a2::TiagoResultConstPtr result = move_to(waypoint_place, ac);

    // Activate the camera to detect the color of the objects and choose the right place
    group_04_a2::CameraResultConstPtr camera_res = cameraDetection(true);
    //Print the ids
    for(int i = 0; i < camera_res->ids.size(); i++){
        ROS_INFO("id %d", camera_res->ids[i]);
    }
    */
    // Fuse the informations together to find the right place

    // Move to the right place
    //move_to(pose_place, ac);

    //Place the object
    //pick_place(camera_res->poses, camera_res->ids, false);
    


    return 0;
}

//***IMPLEMENTATIONS


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

    // Waypoint 2
    waypoint_2.header.stamp = ros::Time::now();
    waypoint_2.header.frame_id = "";
    waypoint_2.pose.position.x = 9.35;
    waypoint_2.pose.position.y = -4.22;
    waypoint_2.pose.position.z = 0;
    waypoint_2.pose.orientation.x = 0;
    waypoint_2.pose.orientation.y = 0;
    waypoint_2.pose.orientation.z = -0.702;

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
#include <group_04_a2/client.h>

// Function to initialize poses
void InitializePoses();
geometry_msgs::PoseStamped pose_1;
geometry_msgs::PoseStamped pose_2;
geometry_msgs::PoseStamped pose_3;
geometry_msgs::PoseStamped waypoint_1;
geometry_msgs::PoseStamped waypoint_2;
geometry_msgs::PoseStamped waypoint_place;

std::vector<geometry_msgs::PoseStamped> computeBarrelPose(geometry_msgs::PoseStamped barrel_wrt_robot, geometry_msgs::PoseStamped wp, int direction)
{
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped to_push;
    tf2::Quaternion q;

    double delta_x = barrel_wrt_robot.pose.position.x;
    double delta_y = barrel_wrt_robot.pose.position.y;

    if(direction == 2) //diagonal right
    {
        to_push.pose.position.x = wp.pose.position.x - delta_y + 0.25;
        to_push.pose.position.y = wp.pose.position.y;
        to_push.pose.position.z = 0;
        q.setRPY(0,0,0);
        to_push.pose.orientation.x = q.x();
        to_push.pose.orientation.y = q.y();
        to_push.pose.orientation.z = q.z();
        to_push.pose.orientation.w = q.w();
        path.push_back(to_push);


        to_push.pose.position.x = wp.pose.position.x - delta_y + 0.25;
        to_push.pose.position.y = wp.pose.position.y + delta_x - 0.45;
        to_push.pose.position.z = 0;
        q.setRPY(0,0,M_PI/2);
        to_push.pose.orientation.x = q.x();
        to_push.pose.orientation.y = q.y();
        to_push.pose.orientation.z = q.z();
        to_push.pose.orientation.w = q.w();
        path.push_back(to_push);

        ROS_INFO(" TO GO at position (%f, %f)", to_push.pose.position.x, to_push.pose.position.y);
            
    }
    else if (direction == 1) //up straight
    {
        to_push.pose.position.x = wp.pose.position.x;
        to_push.pose.position.y = wp.pose.position.y + delta_x -0.35;
        to_push.pose.position.z = 0;
        q.setRPY(0,0,M_PI/2);
        to_push.pose.orientation.x = q.x();
        to_push.pose.orientation.y = q.y();
        to_push.pose.orientation.z = q.z();
        to_push.pose.orientation.w = q.w();
        path.push_back(to_push);

        ROS_INFO(" TO GO at position (%f, %f)", to_push.pose.position.x, to_push.pose.position.y);
    }
    else //diagonal left
    {
        to_push.pose.position.x = wp.pose.position.x - delta_y + 0.22;
        to_push.pose.position.y = wp.pose.position.y;
        to_push.pose.position.z = 0;
        q.setRPY(0,0,0);
        to_push.pose.orientation.x = q.x();
        to_push.pose.orientation.y = q.y();
        to_push.pose.orientation.z = q.z();
        to_push.pose.orientation.w = q.w();
        path.push_back(to_push);


        to_push.pose.position.x = wp.pose.position.x - delta_y + 0.22;
        to_push.pose.position.y = wp.pose.position.y + delta_x - 0.35;
        to_push.pose.position.z = 0;
        q.setRPY(0,0,M_PI/2);
        to_push.pose.orientation.x = q.x();
        to_push.pose.orientation.y = q.y();
        to_push.pose.orientation.z = q.z();
        to_push.pose.orientation.w = q.w();
        path.push_back(to_push);

        ROS_INFO(" TO GO at position (%f, %f)", to_push.pose.position.x, to_push.pose.position.y);
    }

    return path;
}

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
    tf2::Quaternion q; q.setRPY(0,0,0);
    group_04_a2::TiagoGoal goal = createGoal(8.5, 0, 0, q.x(), q.y(), q.z(), q.w(), true);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ac.waitForResult();

    // Substitute with the map one once the place works and the cycle is developed
    InitializePoses();
    
    // Go to the first object
    move_to(pose_1, ac);
    
    // Get the camera results
    group_04_a2::CameraResultConstPtr camera_pointer = cameraDetection();

    //Send pick goal to arm
    pick_place(camera_pointer->poses, camera_pointer->ids, true);
    
    // Go to waypoint 1
    move_to(waypoint_1, ac);
    
    // Go to waypoint place
    group_04_a2::TiagoResultConstPtr result = move_to(waypoint_place, ac);

    std::vector<geometry_msgs::PoseStamped> barrel_poses;
    barrel_poses.push_back(result->result_points[2]);   //left barrel pose wrt robot
    barrel_poses.push_back(result->result_points[1]);   //straight barrel pose wrt robot
    barrel_poses.push_back(result->result_points[0]);   //right barrel pose wrt robot

    // Activate the camera to detect the color of the objects and choose the right place
    group_04_a2::CameraResultConstPtr camera_res = cameraDetection(true);

    
    // Fuse the informations together to find the right place
    for(int i = 0; i < camera_res->ids.size(); i++){
        if (camera_res->ids[i] == 1) //if obj to place is blue = 1
        {
            ROS_INFO("putting object to DIRECTION %d", camera_res->ids[i]);
            ROS_INFO("BARREL TO GO at position (%f, %f, %f)", barrel_poses[i].pose.position.x, barrel_poses[i].pose.position.y, barrel_poses[i].pose.position.z);
            //i is the index of direction... originally since blue is 2 in 0,1,2, blue is on the right
            std::vector<geometry_msgs::PoseStamped> pathBarrel = computeBarrelPose(barrel_poses[i], waypoint_place, i);
            for(int j=0; j<pathBarrel.size(); j++)
                move_to(pathBarrel[j], ac);
            break;
        }
    }

    //Place the object
    pick_place(camera_pointer->poses, camera_pointer->ids, false);
    


    return 0;
}

//***IMPLEMENTATIONS


void InitializePoses(){

    tf2::Quaternion q_minus_pihalf; 
    q_minus_pihalf.setRPY(0,0,-M_PI_2);
    tf2::Quaternion q_pihalf; 
    q_pihalf.setRPY(0,0,M_PI_2);

    // Pose 1
    pose_1.header.stamp = ros::Time::now();
    pose_1.header.frame_id = "";
    pose_1.pose.position.x = 8.053;
    pose_1.pose.position.y = -1.982;
    pose_1.pose.position.z = 0;
    pose_1.pose.orientation.x = q_minus_pihalf.x();
    pose_1.pose.orientation.y = q_minus_pihalf.y();
    pose_1.pose.orientation.z = q_minus_pihalf.z();
    pose_1.pose.orientation.w = q_minus_pihalf.w();

    // Pose 2
    pose_2.header.stamp = ros::Time::now();
    pose_2.header.frame_id = "";
    pose_2.pose.position.x = 7.473;
    pose_2.pose.position.y = -2.103;
    pose_2.pose.position.z = 0;
    pose_2.pose.orientation.x = q_minus_pihalf.x();
    pose_2.pose.orientation.y = q_minus_pihalf.y();
    pose_2.pose.orientation.z = q_minus_pihalf.z();
    pose_2.pose.orientation.w = q_minus_pihalf.w();


    // Pose 3
    pose_3.header.stamp = ros::Time::now();
    pose_3.header.frame_id = "";
    pose_3.pose.position.x = 7.883;
    pose_3.pose.position.y = -3.943;
    pose_3.pose.position.z = 0;
    pose_3.pose.orientation.x = q_pihalf.x();
    pose_3.pose.orientation.y = q_pihalf.y();
    pose_3.pose.orientation.z = q_pihalf.z();
    pose_3.pose.orientation.w = q_pihalf.w();

    // Waypoint 1
    waypoint_1.header.stamp = ros::Time::now();
    waypoint_1.header.frame_id = "";
    waypoint_1.pose.position.x = 8.86;
    waypoint_1.pose.position.y = -2.22;
    waypoint_1.pose.position.z = 0;
    waypoint_1.pose.orientation.x = q_minus_pihalf.x();
    waypoint_1.pose.orientation.y = q_minus_pihalf.y();
    waypoint_1.pose.orientation.z = q_minus_pihalf.z();
    waypoint_1.pose.orientation.w = q_minus_pihalf.w();

    // Waypoint 2
    waypoint_2.header.stamp = ros::Time::now();
    waypoint_2.header.frame_id = "";
    waypoint_2.pose.position.x = 9.35;
    waypoint_2.pose.position.y = -4.22;
    waypoint_2.pose.position.z = 0;
    waypoint_2.pose.orientation.x = q_minus_pihalf.x();
    waypoint_2.pose.orientation.y = q_minus_pihalf.y();
    waypoint_2.pose.orientation.z = q_minus_pihalf.z();
    waypoint_2.pose.orientation.w = q_minus_pihalf.w();

    // Waypoint Place
    waypoint_place.header.stamp = ros::Time::now();
    waypoint_place.header.frame_id = "";
    waypoint_place.pose.position.x = 11.478;
    waypoint_place.pose.position.y = -2.317;
    waypoint_place.pose.position.z = 0;
    waypoint_place.pose.orientation.x = q_pihalf.x();
    waypoint_place.pose.orientation.y = q_pihalf.y();
    waypoint_place.pose.orientation.z = q_pihalf.z();
    waypoint_place.pose.orientation.w = q_pihalf.w();
}
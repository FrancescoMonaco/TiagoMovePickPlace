#include <group_04_a2/arm.h>

//RETURNS COLLISION BOXES FOR EACH SOLID
std::vector<double> Arm::returnDimesions(int id)
{
    //if object is BLUE
    if(id == 1) { return std::vector<double> {0.050301,0.054000,0.1}; }

    //if object is GREEN
    else if(id == 2) { return std::vector<double> {0.06650,0.05500,0.03250};}

    //if object is RED
    else if (id == 3) { return std::vector<double> {0.06800,0.06800,0.05600}; }

    //if object is Gold
    else { return std::vector<double> {0.086602,0.10000, 0.22500}; }
}


void Arm::moveArmPath(const std::vector<geometry_msgs::Pose>& path)
{
    ROS_INFO("-----STARTING PATH------");

    moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_torso");

    move_group_interface.setNumPlanningAttempts(15);
    move_group_interface.setPlanningTime(5);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    for(int i=0; i<path.size(); i++)
    {
        geometry_msgs::Pose step = path[i];
        move_group_interface.setPoseTarget(step);

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
        if (success)
        {
            move_group_interface.execute(my_plan);
            ROS_INFO("ONE STEP DONE");
        }
        else
        {
            ROS_ERROR("FAILED TO PLAN NEXT MOVE --- ABORT");
            break;
        }
    }

    ROS_INFO("-----ENDING PATH------");

    return;
}


//WORK IN PROGRESS - (collision free)
//GO 30 cm above the objects, then ....
void Arm::pickObj(const geometry_msgs::Pose& object, int id)
{
    // Move to a safe pose
    safePose(false);

    if(id == 1){
    //BLUE EXAGON ARM POSE - OK
    std::vector<geometry_msgs::Pose> path_blue;

    tf2::Quaternion q;

    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.z = pose_1.position.z - returnDimesions(id)[1] / 2 +0.20;
    q.setRPY(0, +M_PI/2, 0);
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    path_blue.push_back(pose_1); 
    
     moveArmPath(path_blue);
    
   }
   else if (id == 3){
    //RED CUBE ARM POSES - OK

    std::vector<geometry_msgs::Pose> path_red;

    tf2::Quaternion q;
    geometry_msgs::Pose pose_0;
    pose_0.position = object.position;
    pose_0.position.z = pose_0.position.z - returnDimesions(id)[1] / 2 +0.35;
    pose_0.position.y -= 0.30;
    q.setRPY(0, +M_PI/2, 0);
    pose_0.orientation.x = q.x();
    pose_0.orientation.y = q.y();
    pose_0.orientation.z = q.z();
    pose_0.orientation.w = q.w();
    path_red.push_back(pose_0);

    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.z = pose_1.position.z - returnDimesions(id)[1] / 2 +0.22;
    q.setRPY(0, +M_PI/2, 0);
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    path_red.push_back(pose_1); 
    
    moveArmPath(path_red);
   }
    else {

    //GREEN PYRAMID ARM POSES - THEY SEEMS OK

    std::vector<geometry_msgs::Pose> path_green;

    tf2::Quaternion q;

    geometry_msgs::Pose pose_0;
    pose_0.position = object.position;
    pose_0.position.x -= 0.25;
    pose_0.position.y -= 0.7;
    q.setRPY(0, +M_PI/2, +M_PI/3);
    pose_0.orientation.x = q.x();
    pose_0.orientation.y = q.y();
    pose_0.orientation.z = q.z();
    pose_0.orientation.w = q.w();
    path_green.push_back(pose_0);

    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.x -= 0.25;
    q.setRPY(0, +M_PI/2, +M_PI/4);
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    path_green.push_back(pose_1);

    geometry_msgs::Pose pose_2;
    pose_2.position = object.position;
    pose_2.position.z += 0.30;
    q.setRPY(0, +M_PI/2, +M_PI/3);
    pose_2.orientation.x = q.x();
    pose_2.orientation.y = q.y();
    pose_2.orientation.z = q.z();
    pose_2.orientation.w = q.w();
    path_green.push_back(pose_2);

    geometry_msgs::Pose pose_3;
    pose_3.position = object.position;
    pose_3.position.z += 0.30-0.08;
    q.setRPY(0, +M_PI/2, +M_PI/3);
    pose_3.orientation.x = q.x();
    pose_3.orientation.y = q.y();
    pose_3.orientation.z = q.z();
    pose_3.orientation.w = q.w();
    path_green.push_back(pose_3);


    moveArmPath(path_green);
    }
    return;

}

void Arm::goalCB(const group_04_a2::ArmGoalConstPtr &goal)
{
    auto goal_ = as_.acceptNewGoal();

    //See if we need to pick or place
    bool pick = goal->pick;
    if(pick) pickObject(goal);
    else placeObject(goal);
}

void Arm::pickObject(const group_04_a2::ArmGoalConstPtr &goal)
{
    //Pick the vector of objects and ids
    std::vector<geometry_msgs::Pose> objects = goal->poses;
    std::vector<int> ids = goal->ids;

    // Add the objects to the collision objects
    addCollisionObjects(objects, ids);
    // Move the arm to the object
    pickObj(objects[0], ids[0]);
    // Grip and attach the object
    gripper(false, ids[0]);

    // Tuck again the arm as in the beginning
    safePose(true);

    as_.setSucceeded();
}

void Arm::placeObject(const group_04_a2::ArmGoalConstPtr &goal)
{
}

void Arm::addCollisionObjects(std::vector<geometry_msgs::Pose>& objects, std::vector<int>& ids)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    // Add the objects to the collision objects
    for(int i = 0; i < objects.size(); i++)
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_footprint";
        collision_object.id = "object" + std::to_string(ids[i]);

        // Use a box as the collision object
        shape_msgs::SolidPrimitive object_primitive;
        object_primitive.type = shape_msgs::SolidPrimitive::BOX;
        std::vector<double> dim_s = returnDimesions(ids[i]);
        object_primitive.dimensions.resize(3);

        object_primitive.dimensions[0] = dim_s[0];
        object_primitive.dimensions[1] = dim_s[1];
        object_primitive.dimensions[2] = dim_s[2];

        // Set the pose of the collision object
        geometry_msgs::Pose pose = objects[i];
        if(ids[i] == 2) { pose.position.z -= dim_s[1]/2;}
        else { pose.position.z -= dim_s[1]; }

        pose.orientation = objects[i].orientation;

        collision_object.primitive_poses.push_back(pose);
        collision_object.primitives.push_back(object_primitive);
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(collision_object);
    }

    // Add the table to the collision object
    moveit_msgs::CollisionObject table_coll_object;
    table_coll_object.header.frame_id = "map";
    table_coll_object.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.934400; // x dimension
    primitive.dimensions[1] = 0.934400;  // y dimension
    primitive.dimensions[2] = 0.774700;  // z dimension

    geometry_msgs::Pose pose;
    pose.position.x = 1.2451+6.55;
    pose.position.y = -1.6131-1.35;
    pose.position.z = primitive.dimensions[2] / 2; //NOT 0; IDK, MY TABLE SEEMS HALF IN THE GROUND

    table_coll_object.primitives.push_back(primitive);
    table_coll_object.primitive_poses.push_back(pose);
    table_coll_object.operation = moveit_msgs::CollisionObject::ADD;

    collision_objects.push_back(table_coll_object);
    
    // Add the collision object to the planning scene
    planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void Arm::gripper(bool open, int id)
{
    // Remove the object from the collision objects
    std::vector<std::string> object_ids;
    object_ids.push_back("object" + std::to_string(id));
    planning_scene_interface_.removeCollisionObjects(object_ids);

    if (open)
    {
        detachObjectFromGripper(id);
    }
    else
    {
        attachObjectToGripper(id);
    }
    // Create a publisher for the gripper command
    ros::Publisher gripperPub = nh_.advertise<trajectory_msgs::JointTrajectory>("/parallel_gripper_controller/command", 10);

    // Create a message for the gripper command
    trajectory_msgs::JointTrajectory gripperMsg;
    gripperMsg.header.stamp = ros::Time::now();
    gripperMsg.joint_names.push_back("parallel_gripper_joint");
    trajectory_msgs::JointTrajectoryPoint gripperPoint;
    gripperPoint.positions.push_back(open ? 0.04 : 0.0);
    gripperPoint.time_from_start = ros::Duration(0.5);
    gripperMsg.points.push_back(gripperPoint);

    // Publish the gripper command
    gripperPub.publish(gripperMsg);

    // Perform object attachment/detachment based on the gripper state


    // Close the publisher
    gripperPub.shutdown();
}

void Arm::attachObjectToGripper(int id)
{
    // Create a service client for attaching objects
    attachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

    // Create an Attach service message
    gazebo_ros_link_attacher::Attach attachSrv;
    // Robot model
    attachSrv.request.model_name_1 = "tiago"; 
    attachSrv.request.link_name_1 = "arm_7_link"; 
    // Object model
    if (id == 1){
        attachSrv.request.model_name_2 = "Hexagon";
        attachSrv.request.link_name_2 = "Hexagon_link";
    }
    else if (id == 2){
        attachSrv.request.model_name_2 = "Triangle";
        attachSrv.request.link_name_2 = "Triangle_link";
    }
    else if (id == 3){
        attachSrv.request.model_name_2 = "cube";
        attachSrv.request.link_name_2 = "cube_link";
    }

    // Call the Attach service
    if (attachClient_.call(attachSrv))
    {
        ROS_INFO("Object attached to arm_7_link");
    }
    else
    {
        ROS_ERROR("Failed to attach object to arm_7_link");
    }
}

void Arm::detachObjectFromGripper(int id)
{
    // Create a service client for detaching objects
    detachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // Create a Detach service message
    gazebo_ros_link_attacher::Attach detachSrv;
    // Robot model
    detachSrv.request.model_name_1 = "tiago"; 
    detachSrv.request.link_name_1 = "arm_7_link"; 
    // Object model
    if (id == 1){
        detachSrv.request.model_name_2 = "Hexagon";
        detachSrv.request.link_name_2 = "Hexagon_link";
    }
    else if (id == 2){
        detachSrv.request.model_name_2 = "Triangle";
        detachSrv.request.link_name_2 = "Triangle_link";
    }
    else if (id == 3){
        detachSrv.request.model_name_2 = "cube";
        detachSrv.request.link_name_2 = "cube_link";
    }

    // Call the Detach service
    if (detachClient_.call(detachSrv))
    {
        ROS_INFO("Object detached from arm_7_link");
    }
    else
    {
        ROS_ERROR("Failed to detach object from arm_7_link");
    }
}

void Arm::safePose(bool tuck){
        
    // Go up again before tucking
    moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_torso");
    // Print the number of joints in the arm
    ROS_INFO("Arm with %d joints", joint_model_group->getVariableCount());
    
    move_group_interface.setNumPlanningAttempts(15);
    move_group_interface.setPlanningTime(5);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Set the initial position
    std::vector<double> initial_position = {0.140, 4 * (M_PI / 180), 45 * (M_PI / 180), -80 * (M_PI / 180), 33 * (M_PI / 180), -90 * (M_PI / 180), 78 * (M_PI / 180), 0};
    move_group_interface.setJointValueTarget(initial_position);

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.execute(my_plan);
        ROS_INFO("Initial position done");
    }
    else
    {
        ROS_ERROR("FAILED TO PLAN INITIAL POSITION --- ABORT");
        return;
    }

    if(tuck){
        // Tuck again the arm as in the beginning
        moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_torso");
        // Print the number of joints in the arm
        ROS_INFO("Arm with %d joints", joint_model_group->getVariableCount());
        
        move_group_interface.setNumPlanningAttempts(15);
        move_group_interface.setPlanningTime(5);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // Set the initial position
        std::vector<double> initial_position = {0.185, 11 * (M_PI / 180), -84 * (M_PI / 180), -20 * (M_PI / 180), 103 * (M_PI / 180), -90 * (M_PI / 180), 80 * (M_PI / 180), 0};

        move_group_interface.setJointValueTarget(initial_position);

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_interface.execute(my_plan);
            ROS_INFO("Arm tucked");
        }
        else
        {
            ROS_ERROR("FAILED TO TUCK ARM --- ABORT");
            return;
        }
    }
}
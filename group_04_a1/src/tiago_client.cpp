#include <group_04_a1/client.h>

//*** Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_client");
    // Take from the input the pose (x,y,z, t1, t2, t3) of the goal position
    double x, y, z, t1, t2, t3;
    ros::NodeHandle fh;
    fh.getParam("/tiago_client/x", x);
    fh.getParam("/tiago_client/y", y);
    fh.getParam("/tiago_client/z", z);
    fh.getParam("/tiago_client/t1", t1);
    fh.getParam("/tiago_client/t2", t2);
    fh.getParam("/tiago_client/t3", t3);

    // Initialize the action client
    actionlib::SimpleActionClient<group_04_a1::TiagoAction> ac("tiago_pose", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // Set the goal position
    group_04_a1::TiagoGoal goal = createGoal(x, y, z, t1, t2, t3);
    
    // Send the goal position
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    // Subscribe to the topic that publishes the feedback
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("tiago_pose", 1);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    ros::Rate loop_rate(0.2);
    while(ros::ok()){
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
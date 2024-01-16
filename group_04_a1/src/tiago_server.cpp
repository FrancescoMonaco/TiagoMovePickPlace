#include <group_04_a1/server.h>

//*** Main
int main(int argc, char** argv){
    ros::init(argc, argv, "tiago_pose");

    Tiago tiago(ros::this_node::getName());
    ros::spin();

    return 0;
}


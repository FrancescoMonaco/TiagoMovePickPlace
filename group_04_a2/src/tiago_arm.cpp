#include <group_04_a2/arm.h>

//*** Main
int main(int argc, char** argv){

  ros::init(argc, argv, "tiago_arm");

  Arm object_mover(ros::this_node::getName());
  ros::spin();

  return 0;
}
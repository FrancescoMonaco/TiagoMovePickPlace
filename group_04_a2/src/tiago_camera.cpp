#include <group_04_a2/camera.h>

//*** Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_camera");

  Camera object_detection(ros::this_node::getName());
  ros::spin();

  return 0;
}
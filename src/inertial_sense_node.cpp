#include "inertial_sense_ros.h"

int main(int argc, char**argv)
{
  ros::init(argc, argv, "inertial_sense_node");
  InertialSenseROS thing;
  while (ros::ok())
  {
    ros::spinOnce();
    thing.update();
  }
  return 0;
}
#include <icp_slam/icp_slam.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_slam_node");

  ICPSlam icp_slam;

  ros::spin();

  return 0;
}

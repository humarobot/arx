
#include "demonstrator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultron_demonstrate_mode");
  ros::NodeHandle nh;
  Demonstrator demonstrator(nh);
  demonstrator.StartUp(500);
  return 0;
}

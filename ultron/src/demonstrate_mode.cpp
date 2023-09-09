
#include "demonstrator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultron_demonstrate_mode");
  ros::NodeHandle nh;
  Demonstrator demonstrator(nh,500,"test.bag");
  demonstrator.StartUp();
  // demonstrator.Record();
  // demonstrator.Replay(); 
  demonstrator.ReplayImpedence();
  
  return 0;
}

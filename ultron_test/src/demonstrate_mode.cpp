
#include "demonstrator.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultron_demonstrate_mode");
  ros::NodeHandle nh;
  Demonstrator demonstrator(500,"test.bag");
  // ros::Subscriber command_sub = nh.subscribe("/demons trator",10,&callback);
  ros::Subscriber command_sub = nh.subscribe("/demonstrator",10,&Demonstrator::Callback,&demonstrator);
  ros::Rate loop(100);
  demonstrator.StartUp();
  // demonstrator.Record();
  // demonstrator.Replay(); 
  while(ros::ok()){
    if(demonstrator.command == StartRecord){
      demonstrator.Record();
    }else if(demonstrator.command == StartReplay && demonstrator.GetRecording()==false){
      demonstrator.ReplayImpedence();
    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}


#include "demonstrator.hpp"
#include "std_msgs/Int32.h"

enum Command {
  StartRecord,
  StopRecord,
  StartReplay,
  StopReplay,
  Idle
};
Command command;
ros::NodeHandle nh;
Demonstrator demonstrator(nh,500,"test.bag");

void callback(const std_msgs::Int32::ConstPtr msg){
  switch (msg->data){
    case 0:
      command = Idle;
      break;
    case 1:
      command = StartRecord;
      demonstrator.SetRecording(true);
      break;
    case 2:
      command = StopRecord;
      demonstrator.SetRecording(false);
      break;
    case 3:
      command = StartReplay;
      break;
    case 4:
      command = StopReplay;
      break;
    default:
      break;
  }

  std::cout<<command<<std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultron_demonstrate_mode");
  ros::Subscriber command_sub = nh.subscribe("/demonstrator",10,callback);
  ros::Rate loop(100);
  command = Idle;
  demonstrator.StartUp();
  // demonstrator.Record();
  // demonstrator.Replay(); 
  while(ros::ok()){
    if(command == StartRecord){
      demonstrator.Record();
    }else if(command == StartReplay && demonstrator.GetRecording()==false){
      demonstrator.ReplayImpedence();
    }
    ros::spinOnce();
    loop.sleep();
  }

  
  return 0;
}

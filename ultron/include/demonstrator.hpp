#include "App/arm_control.h"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>


class Demonstrator
{
public:
  Demonstrator(ros::NodeHandle& nh, int hz,const std::string& bag_name);
  ~Demonstrator() = default;

  void StartUp();
  void Record();
  void Replay();
  void Pause();
  void Stop();

private:
  std::shared_ptr<arx_arm> robotic_arm_;
  ros::NodeHandle& nh_;
  Eigen::VectorXd tau_w_{6},q_r_{6},v_r_{6};
  int hz_;
  rosbag::Bag bag_;
  const std::string bag_name_;

};
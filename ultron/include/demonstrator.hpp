#include "App/arm_control.h"
#include "ros/ros.h"


class Demonstrator
{
public:
  Demonstrator(ros::NodeHandle& nh);
  ~Demonstrator() = default;

  void StartUp(int hz);
  void Record();
  void Replay();
  void Pause();
  void Stop();

private:
  std::shared_ptr<arx_arm> robotic_arm_;
  ros::NodeHandle& nh_;
  Eigen::VectorXd tau_w_{6},q_r_{6},v_r_{6};

};
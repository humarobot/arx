#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "App/arm_control.h"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64MultiArray.h>


class Demonstrator
{
public:
  Demonstrator(ros::NodeHandle& nh, int hz,const std::string& bag_name);
  ~Demonstrator() = default;

  void StartUp();
  void Record();
  void Replay();
  void ReplayImpedence();
  void Pause();
  void Stop();

private:
  std::shared_ptr<arx_arm> robotic_arm_;
  ros::NodeHandle& nh_;
  Eigen::VectorXd tau_w_{6},q_r_{6},v_r_{6};
  int hz_;
  rosbag::Bag bag_;
  const std::string bag_name_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  int link_id_;

};
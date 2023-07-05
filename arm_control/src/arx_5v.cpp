#include "App/arm_control.cpp"
#include "App/arm_control.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "arm_control/Motor.h"
#include "utility.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define IS_REAL 0

int CONTROL_MODE = 3;
// int can_cmd_init_flag=0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot"); // name is overridden by roslaunch
  ros::NodeHandle node;
  arx_arm ARX_ARM((bool)IS_REAL);
  ros::Rate loop_rate(200);
  // can CAN_Handlej;

  while (ros::ok()) {
    ARX_ARM.get_curr_pos();
    // Motor Setting 0 position
    // ARX_ARM.RealInit();
    // ARX_ARM.update_real();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
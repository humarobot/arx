#ifndef _ISAAC_GIMBAL_H_
#define _ISAAC_GIMBAL_H_

#include "../Hardware/can.h"
#include "../Hardware/motor.h"
#include "Eigen/Core"
#include "Hardware/teleop.h"
#include "utility.h"
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <memory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

//
#define ARX5_PRO_L1 0.27
#define ARX5_PRO_L2 0.2766
#define A4310_kp 160
#define A4310_kD 5
// #define spd 100
// #define cur 1000
#define YAW_WORLD 0
#define BASE 1
#define ELBOW 2
#define PITCH_WAIST 3
#define YAW_WAIST 4
#define ROLL 5
#define GRASP 6

#define X 0
#define Y 1
#define Z 2

// Number of joints
#define NJ 6

#define FORWARD 0
#define DOWNWARD 1

#define JOYSTICK_DEADZONE 0.15

#define SIM_JOINT_KP 150
#define SIM_JOINT_KD 10

#define filter_sensor 0.3f
#define filter_vel 0.3f
#define filter_cmd 0.3f
#define filter_torque 0.3f
#define encos_up 2.09f
#define encos_down -2.09f

/* currently, one mission one class, if there exists multi mission and share arm
variables, either use a more gerenal class including mission instance or use
global struct
*/

struct command {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float base_yaw = 0.0f;
  float gripper_roll = 0.0f;
  float gripper = 0.0f;
  float waist_yaw = 0.0f;
  float waist_pitch = 0.0f;
  bool reset = false;
  int mode = FORWARD;
};

struct cartesian {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct cylinder {
  float r = 0.0f;
  float phi = 0.0f;
  float z = 0.0f;
};

struct sphere {
  float rho = 0.0f;
  float phi = 0.0f;
  float theta = 0.0f;
};

struct coordinate {
  cartesian xyz_pos;
  cylinder cylinder_pos;
  sphere sphere_pos;
};

enum arx5_state { NORMAL, OUT_RANGE, OUT_BOUNDARY };

class FIFO_Queue {
public:
  void write_data(geometry_msgs::PoseStamped pos_stamped);
  geometry_msgs::PoseStamped read_nearest_data(ros::Time time);
  uint count = 0;
  std::vector<geometry_msgs::PoseStamped> data;
  uint write_ptr = 0, read_ptr = 0;
};

class arx_arm {
public:
  arx_arm(bool real_flg);
  ~arx_arm() = default;

  bool is_real = false;
  arx5_state command_state;

  unsigned int loop_rate = 200;

  float current_pos[7] = {0.0f};
  float current_vel[7] = {0.0};
  float target_pos[7] = {0.0f}, last_target_pos[7] = {0.0f};
  float target_vel[7] = {0.0f};

  // For limiting the orientation of the motors
  float pitch_waist_lb = -1.72;
  float lower_bound_sim[7] = {encos_down, encos_down, encos_down, encos_down,
                              encos_down, encos_down, -3.14};
  float upper_bound_sim[7] = {encos_up, encos_up, encos_up, encos_up,
                              encos_up, encos_up, 3.14};

  float lower_bound_waist[3] = {0.03, -0.6, -0.6};
  float upper_bound_waist[3] = {0.6, 0.6, 0.6};

  float lower_bound_pitch = 0;
  float upper_bound_pitch = M_PI / 2;
  float lower_bound_yaw = -1.35;
  float upper_bound_yaw = 1.35;

  float max_torque = 15;

  // Save the coordinates on previous round
  float prev_x = 0, prev_y = 0, prev_z = 0, prev_roll = 0, prev_pitch = 0,
        prev_yaw = 0, prev_base_yaw = 0;

  float yaw_base2waist = 0.0f;
  float yaw_world2base = 0.0f;

  float ramp(float final, float now, float ramp_k);
  float joystick_projection(float joy_axis);

  // Get the actual position of joints from CAN controller in real mode
  void get_curr_pos(Eigen::VectorXd& p, Eigen::VectorXd& v);
  void get_curr_pos();
  void set_joints_pos(const Eigen::VectorXd& pos);
  void set_tail_3_pos(const Eigen::VectorXd& pos);
  void set_head_3_torque(const Eigen::VectorXd& torque);
  void set_joints_torque(const Eigen::VectorXd& torque);
  void set_loop_rate(const unsigned int rate);
  // For startup process
  void init_step();
  bool is_starting = true;
  // KDL::JntArray init_dest_pos = KDL::JntArray(NJ);

  // Inverse kinamics calculation parameters
  KDL::ChainIkSolverPos_LMA *iksolver;
  KDL::ChainFkSolverPos_recursive *FKSolver;
  KDL::ChainFkSolverVel_recursive *FKVelSolver;
  KDL::ChainIkSolverVel_pinv *IKAccSolver;
  KDL::ChainJntToJacDotSolver *JacDotSolver;
  KDL::Frame initFrame;
  KDL::FrameVel curr_frame;
  KDL::Tree tree;
  KDL::Chain chain;
  KDL::JntArray jointGuesspositions = KDL::JntArray(NJ);
  KDL::JntArrayVel jointCurrentVel = KDL::JntArrayVel(NJ);
  KDL::Twist targetCartAcc, tempTwist;

  unsigned int nj; // number of joints
  double eps = 1E-5;
  int maxiter = 500;
  double eps_joints = 1E-15;
  std::string model_path;
  arx5_state kdl_ik_calc();

  // Inverse dynamics calculation parameters

  void kdl_id_calc();
  void teach2pos_mode_step();
  float prev_target_pos[NJ] = {0};
  bool teach2pos_returning = false;
  KDL::JntArray jointPrevPositions = KDL::JntArray(NJ);
  KDL::JntArray jointPrevVelocities = KDL::JntArray(NJ);
  KDL::JntArray jointVelocities = KDL::JntArray(NJ);
  KDL::JntArray jointAcclerations = KDL::JntArray(NJ);
  KDL::JntArray jointTorques = KDL::JntArray(NJ);
  KDL::JntArray jointPrevTorques = KDL::JntArray(NJ);
  float motor_torque_k = 0.72; // 0.8  0.75

  float kx = 100;
  float ky = 100;
  float kz = 400;
  float kroll = 0;
  float kpitch = 400;
  float kyaw = 100;

  float kdx = 20;
  float kdy = 20;
  float kdz = 20;
  float kdroll = 0;
  float kdpitch = 20;
  float kdyaw = 10;

  float kiz = 0.000000001;
  float kipitch = 0.000000001;
  double intergral_z = 0;
  double intergral_pitch = 0;

  KDL::Frame prev_frame;
  // void calc_gripper_force();
  void calc_joint_acc();
  float f_x = 0, f_y = 0, f_z = 0, t_r = 0, t_p = 0, t_y = 0;

  ros::NodeHandle nh;
  ros::Publisher pos_pub =
      nh.advertise<geometry_msgs::Vector3>("armcontrol/current_pos", 10);
  ros::Publisher vel_pub =
      nh.advertise<geometry_msgs::Vector3>("armcontrol/current_vel", 10);
  ros::Publisher ang_pub =
      nh.advertise<geometry_msgs::Vector3>("armcontrol/current_angle", 10);
  // ros::Publisher pub_posY = nh.advertise<float>("armcontrol/posY", 10);

  // For random position control
  bool use_random = false;
  const int pause_time = 400; // Step per pause
  void set_rand_joint_pos();
  bool check_valid_rand_pos(KDL::JntArray jointpos);
  void rand_step();
  bool is_rand_moving = false;
  bool is_rand_mode = false;
  bool is_rand_return = false;
  int curr_pause_time = 0;
  bool button5_pressed = false;
  float gripper_rand_pos = 0;
  KDL::JntArray rand_dest_pos = KDL::JntArray(NJ + 1);

  ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>(
      "armcontrol/current_joint_angle", 10);

  // For ROS Control
  bool is_ros_control = false;
  geometry_msgs::Pose target_pose; // 注意是相机系
  geometry_msgs::Pose ctrl_target_pose;
  float ros_control_filter = 0.0;
  double k_ros_ctrl = 0.01;
  void ros_ctrl_step();
  // void target_tf_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // ros::Subscriber target_tf_sub =
  // nh.subscribe<geometry_msgs::PoseStamped>("target_pose", 1000,
  // &arx_arm::target_tf_Callback, this);

  geometry_msgs::PoseStamped curr_pos_stamped,
      target_pos_stamped; // 注意是相机系

  // For recording the movements
  bool button3_pressed = 0;
  bool button0_pressed = 0;

  bool is_torque_control = false;
  bool is_teach_mode = false;

  bool teach_mode = false;
  void end_record();
  void update_record();

  bool read_write_multiple_files = true;
  /////////////////////////////////////////////////////////////

  void unify_coordinate(); // Old unused function

  command get_cmd(); // ask for teleop cmd
  command arx5_cmd;
  float sim_Kp[7] = {SIM_JOINT_KP, SIM_JOINT_KP, SIM_JOINT_KP, SIM_JOINT_KP,
                     SIM_JOINT_KP, SIM_JOINT_KP, SIM_JOINT_KP};
  float sim_Kd[7] = {SIM_JOINT_KD, SIM_JOINT_KD, SIM_JOINT_KD, SIM_JOINT_KD,
                     SIM_JOINT_KD, SIM_JOINT_KD, SIM_JOINT_KD};

  int gimbal_mode = FORWARD;

  bool is_PD_control = true;
  bool is_test_mode = false;

  // #real_joint
  int set_sw = 0;
  float set_use_kp = 200;   // 200
  float set_use_kd = 10;    // 10
  float set_use_kp15 = 260; // 260
  float set_use_kd15 = 10;  // 10
  float set_use_kp26 = 180; // 200
  float set_use_kd26 = 10;  // 10

  float tgt_pos[8] = {(6.357f - 0.1f), 1.221,  -0.734f, -0.744f,
                      (6.137f + 0.1f), 4.997f, 7.041f,  -0.423f}; // 平+衡车模式
  float tgt_pos_test[6] = {0.0f, -60.0, 82.12f, -21.0f, 0.0f, 0.0f};

  float k_imu = 0.045f; // 0.06
  float total_set_imu = 0;
  float Kps[8] = {200.0f, 0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f, 0.0f};
  float Kds[8] = {10.0f, 15.0f, 10.0f, 10.0f, 10.0f, 15.0f, 10.0f, 10.0f};
  float zeros8[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  float Kp[8] = {A4310_kp, A4310_kp, A4310_kp, A4310_kp,
                 A4310_kp, A4310_kp, A4310_kp, A4310_kp};
  float Kd[8] = {A4310_kD, A4310_kD, A4310_kD, A4310_kD,
                 A4310_kD, A4310_kD, A4310_kD, A4310_kD};
  // float Kp[8] = {0};
  // float Kd[8] = {0};
  // uint16_t tgt_spd[8] = {0};
  float ffw_torque[8] = {0.0f};
  // float tgt_poss[8] = {0.0f};
  float tgt_spds[8] = {0.0f};
  float ffw_torques[8] = {0.0f};
  float body_kp = 15.0f, body_kd = 1.0f;

  float speed_kpj = 1, speed_kij = 0.0f, fpj, fij, wheel_speedj, total_fij;
  // float body_kp=1.0f,body_kd=0.3f;
  float v2b_bodycontrol(int safe, int sw2, int ch_1, float imu_pit_gyro,
                        float wheel_speed1, float wheel_speed2);
  float v2b_out = 0;
  void safe_model(void);
  void set_zero(void);
  void init_moto_pos(void);
  // void v2b_bodycontrol(void);
  void body_balance_UP(void);
  void body_balance(void);
  void update(void);
  void update_real();
  void RealInit();
  void motor_control();
  void can_position_control(uint16_t motor_id, float pos, uint16_t spd,
                            uint16_t cur, uint8_t ack_status);
  // Interface
  // bool is_real = 0;
  void control_4005();
  int init_end = 0, add_pos = 0;
  float set_4005_out = 0, set_pos_4005 = 0, k1_4005 = 400, k2_4005 = 3,
        init_pos_4005 = 0, pos_4005_t = 0;
  can CAN_Handlej;

private:
  // for ros configurations
  // currently not necessary

  ros::Publisher joint_state_pub;
  sensor_msgs::JointState joint_state;

  ros::Publisher joint_cmd_pub;
  sensor_msgs::JointState joint_cmd;

  float test_pos = 0;
  uint test_cnt = 0;

  float pos_filted[3], vel_filted[3];
};

#endif
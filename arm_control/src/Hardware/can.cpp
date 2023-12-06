#include "Hardware/can.h"

m_rmd_t rmd_9015_01 = {0};
m_rmd_t rmd_9015_02 = {0};
m_rmd_t rmd_01 = {0};
IMU_Float_t imu_c_board = {0};

arm_control::Imu g_imu_msg;
arm_control::Motor g_wheel1_msg;
arm_control::Motor g_wheel2_msg;

void CAN2_ReceiveHandlerProxy(can_frame_t *frame, void *ptr) {
  ((can *)ptr)->CAN2_ReceiveFrame(frame);
}

can::can() {
  can2_adapter.reception_handler_data = (void *)this;
  can2_adapter.reception_handler = &CAN2_ReceiveHandlerProxy;
  can2_adapter.open("can2");

  ros::Time::init();
}

can::~can() {
  can2_adapter.close();
}

void can::CAN_cmd_readMotorID(void) {
  can_frame_t frame;
  MotorIDReading(frame.data, &frame.can_id, &frame.can_dlc);
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

void can::CAN_cmd_getMotorParam(uint16_t motor_id, uint8_t param_cmd) {
  can_frame_t frame;
  GetMotorParameter(motor_id, param_cmd, frame.data, &frame.can_id,
                    &frame.can_dlc);
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

void can::CAN_cmd_init(uint16_t motor_id, uint8_t cmd) {

  can_frame_t frame;
  frame.can_dlc = 8;
  MotorSetting(motor_id, cmd);
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}
void can::Can_cmd_position(uint16_t motor_id, float pos, uint16_t spd,
                           uint16_t cur, uint8_t ack_status) {
  can_frame_t frame;
  frame.can_dlc = 8;
  send_motor_position(motor_id, pos, spd, cur, ack_status, frame.data,
                      &frame.can_id);
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

void can::Can_cmd_all(uint16_t motor_id, float kp, float kd, float pos,
                      float spd, float tor) {
  can_frame_t frame;
  frame.can_dlc = 8;
  send_motor_ctrl_cmd(motor_id, kp, kd, pos, spd, tor, frame.data,
                      &frame.can_id);
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

// Motor motor
// motor id 0x01 ~ 0x04
void can::CAN_cmd_fpc1(float kp[4], float kd[4], float pos[4], float spd[4],
                       float tor[4]) {
  for (int i = 0; i < 4; i++) {
    can_frame_t frame;
    frame.can_dlc = 8;
    send_motor_ctrl_cmd(i + 1, kp[i], kd[i], pos[i], spd[i], tor[i], frame.data,
                        &frame.can_id);
    if (can2_adapter.is_open()) {
      can2_adapter.transmit(&frame);
    } else {
      std::cout << "Fail to open can2" << std::endl;
    }
  }
}

// Motor motor
// motor id 0x05 ~ 0x08
void can::CAN_cmd_fpc2(float kp[4], float kd[4], float pos[4], float spd[4],
                       float tor[4]) {
  for (int i = 0; i < 4; i++) {
    can_frame_t frame;
    frame.can_dlc = 8;
    send_motor_ctrl_cmd(i + 5, kp[i], kd[i], pos[i], spd[i], tor[i], frame.data,
                        &frame.can_id);
    if (can2_adapter.is_open()) {
      can2_adapter.transmit(&frame);
    } else {
      std::cout << "Fail to open can2" << std::endl;
    }
  }
}

// // Motor 8120
// // motor id 0x01 ~ 0x04
// void can::CAN_cmd_chassis1(int16_t motor[4])
// {
//     can_frame_t frame;
//     frame.can_id = 0x1FF;
//     frame.can_dlc = 8;
//     for (int i = 0; i < 4; i++)
//     {
//         frame.data[2 * i] = motor[i] >> 8;
//         frame.data[2 * i + 1] = motor[i] & 0xFF;
//     }
//     if (can0_adapter.is_open())
//     {
//         can0_adapter.transmit(&frame);
//     }
//     else
//     {
//         std::cout << "Fail to open can0" << std::endl;
//     }
// }

// // Motor 8120
// // motor id 0x05 ~ 0x08
// void can::CAN_cmd_chassis2(int16_t motor[4])
// {
//     can_frame_t frame;
//     frame.can_id = 0x2FF;
//     frame.can_dlc = 8;
//     for (int i = 0; i < 4; i++)
//     {
//         frame.data[2 * i] = motor[i] >> 8;
//         frame.data[2 * i + 1] = motor[i] & 0xFF;
//     }
//     if (can0_adapter.is_open())
//     {
//         can0_adapter.transmit(&frame);
//     }
//     else
//     {
//         std::cout << "Fail to open can0" << std::endl;
//     }
// }

void can::GetImuAngle(uint8_t *data) {
  imu_c_board.last_imu[0] = imu_c_board.imu[0];
  imu_c_board.last_imu[2] = imu_c_board.imu[2];
  imu_c_board.last_imu[1] = imu_c_board.imu[1];

  imu_c_board.imu[0] = (int16_t)(data[0] << 8 | data[1]) * M_PI / 32768;
  imu_c_board.imu[2] = (int16_t)(data[2] << 8 | data[3]) * M_PI / 32768;
  imu_c_board.imu[1] = (int16_t)(data[4] << 8 | data[5]) * M_PI / 32768;

  // float yaw = (int16_t)(data[0] << 8 | data[1]) * M_PI / 32768;
  // float roll = (int16_t)(data[2] << 8 | data[3]) * M_PI / 32768;
  // float pitch = (int16_t)(data[4] << 8 | data[5]) * M_PI / 32768;

  // std::cout << "yaw: " << yaw << " roll: " << roll << " pitch: " << pitch <<
  // std::endl; ImuMsg.orientation =
  // tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  // //实际是角度，欧拉角, rad
  // ImuMsg.linear_acceleration.x = yaw;
  // ImuMsg.linear_acceleration.y = roll;
  // ImuMsg.linear_acceleration.z = pitch;

  // ImuMsg.header.stamp = ros::Time::now();
  // freq++; //计数器
}

void can::GetImuGyro(uint8_t *data) {

  imu_c_board.gyro[0] = (int16_t)(data[0] << 8 | data[1]) * 50.0 / 32768;
  imu_c_board.gyro[1] = (int16_t)(data[2] << 8 | data[3]) * 50.0 / 32768;
  imu_c_board.gyro[2] = (int16_t)(data[4] << 8 | data[5]) * 50.0 / 32768;

  // // std::cout << (int16_t)(data[0] << 8 | data[1]) << std::endl;
  // ImuMsg.angular_velocity.x = (int16_t)(data[0] << 8 | data[1]) * 50.0 /
  // 32768; ImuMsg.angular_velocity.y = (int16_t)(data[2] << 8 | data[3]) * 50.0
  // / 32768; ImuMsg.angular_velocity.z = (int16_t)(data[4] << 8 | data[5])
  // * 50.0 / 32768; ImuMsg.header.stamp = ros::Time::now();
}

/**
 * @description: 收到can2的消息
 */
void can::CAN2_ReceiveFrame(can_frame_t *frame) {

  switch (frame->can_id) {
  case 0x01:
  case 0x02:
  case 0x03:
  case 0x04:
  case 0x05:
  case 0x06:
  case 0x07:
  case 0x08:
  case 0x7FF: {
    RV_can_data_repack(frame->can_id, frame->data, frame->can_dlc, 0);
    break;
  }
  case 0x205:
  case 0x206:
  case 0x207:
  case 0x208:
  case 0x209:
  case 0x20A:
  case 0x20B:
  case 0x20C: {
    // // 电机
    // uint8_t i = frame->can_id - 0x205;
    // if (Motor[i].msg_cnt < 10)
    // {
    //     get_motor_offset(&Motor[i], frame->data);
    //     Motor[i].msg_cnt++;
    // }

    // get_motor_measure(&Motor[i], frame->data);

    // // Motor[i].msg_cnt++ <= 10 ? get_motor_offset(&Motor[i], frame->data) :
    // get_motor_measure(&Motor[i], frame->data);
    break;
  }

  // case 0x141:
  // {
  //     rmd_9015_01.speed_rpm=((int16_t)(frame->data[5]<<8)
  //     |(frame->data[4]))/57.3f; rmd_9015_01.position
  //     =((int16_t)(frame->data[7]<<8) |(frame->data[6])); break;
  // }
  case 0x141: {
    rmd_01.total_angle_last = rmd_01.position;
    rmd_01.speed_rpm =
        ((int16_t)(frame->data[5] << 8) | (frame->data[4])) / 57.3f;
    rmd_01.position = ((int16_t)(frame->data[7] << 8) | (frame->data[6]));
    rmd_01.real_current = ((int16_t)(frame->data[2] << 8) | (frame->data[3]));
    if (rmd_01.position - rmd_01.total_angle_last > 8192)
      rmd_01.round_cnt--;
    else if (rmd_01.position - rmd_01.total_angle_last < -8192)
      rmd_01.round_cnt++;
    rmd_01.total_angle = rmd_01.round_cnt * 16384 + rmd_01.position;
    break;
  }
  case 0x142: {
    rmd_9015_02.speed_rpm =
        ((int16_t)(frame->data[5] << 8) | (frame->data[4])) / 57.3f;
    rmd_9015_02.position = ((int16_t)(frame->data[7] << 8) | (frame->data[6]));
    break;
  }

  case 0x501: {
    // std::cout << "# frame can_id= " << frame->can_id << std::endl;
    // std::cout << "# frame data0-3  = " << frame->data[0] << " " <<
    // frame->data[1] << " " << frame->data[2] << " " << frame->data[3] << " "
    // << std::endl; std::cout << "# frame data4-7  = " << frame->data[4] << " "
    // << frame->data[5] << " " << frame->data[6] << " " << frame->data[7] << "
    // " << std::endl; IMU float tmp[2]; memcpy((uint8_t *)(&tmp[0]), (uint8_t
    // *)(&frame->data[0]), 4); memcpy((uint8_t *)(&tmp[1]), (uint8_t
    // *)(&frame->data[4]), 4); IMU.imu[0] = tmp[0] * 0.0174533; IMU.imu[1] =
    // tmp[1] * 0.0174533;
    break;
  }
  case 0x502: {
    // std::cout << "# frame can_id= " << frame->can_id << std::endl;
    // std::cout << "# frame data0-3  = " << frame->data[0] << " " <<
    // frame->data[1] << " " << frame->data[2] << " " << frame->data[3] << " "
    // << std::endl; std::cout << "# frame data4-7  = " << frame->data[4] << " "
    // << frame->data[5] << " " << frame->data[6] << " " << frame->data[7] << "
    // " << std::endl;
    // // Gyro
    // memcpy((uint8_t *)(&IMU.gyro[0]), (uint8_t *)(&frame->data[0]), 4);
    // memcpy((uint8_t *)(&IMU.gyro[1]), (uint8_t *)(&frame->data[4]), 4);
    break;
  }
  case 0x666: {
    GetImuAngle(frame->data);
  } break;
  case 0x667: {
    GetImuGyro(frame->data);
  } break;
  default: {
    break;
  }
  }
}

//  01  02  03零点设置
void can::MotorSetting(uint16_t motor_id, uint8_t cmd) {
  can_frame_t frame;
  frame.can_id = 0x7FF;
  frame.can_dlc = 4;
  if (cmd == 0)
    return;

  frame.data[0] = motor_id >> 8;
  frame.data[1] = motor_id & 0xff;
  frame.data[2] = 0x00;
  frame.data[3] = cmd;

  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

void can::CAN_RMD_chassis(int16_t motor1, int16_t motor2, int16_t motor3,
                          int16_t motor4) {
  //    std::cout << "》》》》》》" << std::endl;
  can_frame_t frame;
  frame.can_id = 0x280;
  frame.can_dlc = 8;

  // if(motor1>1900); motor1=1900;
  // if(motor1<-1900); motor1=-1900;
  // if(motor2>1900); motor2=1900;
  // if(motor2<-1900); motor2=-1900;

  frame.data[0] = motor1;
  frame.data[1] = motor1 >> 8;
  frame.data[2] = motor2;
  frame.data[3] = motor2 >> 8;
  frame.data[4] = motor3;
  frame.data[5] = motor3 >> 8;
  frame.data[6] = motor4;
  frame.data[7] = motor4 >> 8;
  if (can2_adapter.is_open()) {
    can2_adapter.transmit(&frame);
  } else {
    std::cout << "Fail to open can2" << std::endl;
  }
}

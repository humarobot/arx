#include "ros/ros.h"
#include "serial/serial.h"
#include "string"
#include "std_msgs/Bool.h"
//读取 /hand_state 话题，控制手的开闭

/* define */
#define PROTOCOL1
// #define PROTOCOL2

#define DEVICEID 0x01

#define TORQUEENABLE 64
#define LED 65
#define GOALCURRENT 102

typedef enum {
  DEFAULT = 0,
  PING,
  READ,
  WRITE,
  REGWRITE,
  ACTION,
  FACTORYRESET,
  REBOOT,
  SYNCWRITE,
  BULKREAD
} instruction;

typedef enum { OFF = 0, ON } state;

/* variable */
serial::Serial ser;
uint8_t rxBuffer[256], txBuffer[256];

/* private */
int prvSerialWrite(int len) {
  int re = ser.write(txBuffer, len + 4);
  if (!re)
    printf("serial writing error");
  return re;
}

/* public */
void Encode(uint8_t ID, uint8_t len, instruction ins, uint8_t *data) {
#ifdef PROTOCOL1
  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;
  txBuffer[2] = ID;
  txBuffer[3] = len;
  txBuffer[4] = ins;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len - 2; i++) {
    txBuffer[i + 5] = data[i];
    sum += data[i];
  }
  sum += ID;
  sum += len;
  sum += ins;
  txBuffer[len + 3] = ~sum;
#endif

#ifdef PROTOCOL2
  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;
  txBuffer[2] = 0xFD;
  txBuffer[3] = 0x00;
  txBuffer[4] = ID;
  txBuffer[5] = len;
  txBuffer[6] = 0x00;
  txBuffer[7] = ins;
  for (uint8_t i = 0; i < len - 2; i++) {
    txBuffer[i + 8] = data[i];
  }
  // have not write CRC
#endif
}

int Ping(uint8_t ID) {
  uint8_t nodata;
  uint8_t len = 0x02;
  Encode(ID, len, PING, &nodata);
  ROS_INFO("%d", txBuffer[len + 3]);
  return prvSerialWrite(len);
}

int Write(uint8_t ID, uint8_t len, uint8_t *data) {
  Encode(ID, len, WRITE, data);
  return prvSerialWrite(len);
}

int TorqueEnable(uint8_t ID, state torque_state) {
  uint8_t len = 0x04;
  uint8_t data[2];
  data[0] = TORQUEENABLE;
  data[1] = torque_state;
  return Write(ID, len, data);
}

int Led(uint8_t ID, state led_state) {
  uint8_t len = 0x04;
  uint8_t data[2];
  data[0] = LED;
  data[1] = led_state;
  return Write(ID, len, data);
}

int GoalCurrent(uint8_t ID, int goal_current) {
  uint8_t len = 0x05;
  uint8_t data[3];
  data[0] = GOALCURRENT;
  data[1] = goal_current & 0xff;
  data[2] = goal_current >> 8;
  return Write(ID, len, data);
}

void callback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data){
    GoalCurrent(DEVICEID, 100);
    std::cout<<"hand close"<<std::endl;
  }
  else{
    GoalCurrent(DEVICEID, -50);
    std::cout<<"hand open"<<std::endl;
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_usb");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/hand_state", 10, callback);
  std::string serial_port_;
  int baudrate_;

  n.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  n.param<int>("baudrate", baudrate_, 9600);

  ros::Rate loop(4);

  try {
    ser.setPort(serial_port_);
    ser.setBaudrate(baudrate_);
    serial::Timeout to1 = serial::Timeout::simpleTimeout(1000); // 超时等待
    ser.setTimeout(to1);

    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port!!!");
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized...");
  } else {
    ROS_INFO_STREAM("initialization failed!!!");
    return -1;
  }
  ROS_INFO("ready");

  for(int i=0;i<5;i++){
    TorqueEnable(DEVICEID, ON);
    loop.sleep();
  }
  while(ros::ok()){
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}

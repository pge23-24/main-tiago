// Copyright 2024 cario360

#ifndef ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_
#define ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

extern int serial;

#define PORT "/dev/ttyACM0"
#define BAUDRATE B9600

#define CMD_VEL_TOPIC "/mobile_base_controller/cmd_vel"
#define STOP_DURATION 2.0

#define LED_ON "\x65"
#define LED_OFF "\x6F"

#define STATE_STOPPED 0
#define STATE_MOUVEMENT 1
#define STATE_STOP_WAIT 2

class RelayControlNode {
 public:
  RelayControlNode(const char* port, int baudrate);
  void cmd_vel_callback(const geometry_msgs::Twist& msg);

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub;

  int serial;
  ros::Time timerStart = ros::Time::now();
  int state = STATE_STOPPED;
};

#endif  // ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

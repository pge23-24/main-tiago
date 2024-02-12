// Copyright 2024 cario360

#ifndef ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_
#define ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/RecoveryStatus.h"
#include "ros/ros.h"
#include <signal.h>

extern int serial;

#define PORT "/dev/ttyACM0"
#define BAUDRATE B9600

#define CMD_VEL_TOPIC "/mobile_base_controller/cmd_vel"
#define JOY_TOPIC "/joy_priority_action/status"
#define RECOVERY_TOPIC "/move_base/recovery_status"
#define STOP_DURATION 2.0

#define SPOT_T_ON 0.5
#define SPOT_T_OFF 0.5
#define BUFFER_SIZE 10;

//SPOT = Relay 1
#define SPOT_ON "\x65"
#define SPOT_OFF "\x6F"
//GREEN = Relay 2
#define MAT_GREEN_ON "\x66"
#define MAT_GREEN_OFF "\x70"
//ORANGE = Relay 3
#define MAT_ORANGE_ON "\x67"
#define MAT_ORANGE_OFF "\x71"
//RED = Relay 4
#define MAT_RED_ON "\x68"
#define MAT_RED_OFF "\x72"

#define ALL_OFF "\x6E"

enum SpotState {ETEINT, CLIGNOTANT, ALLUME};
enum BlinkState {B_ETEINT, B_WAIT1, B_ALLUME, B_WAIT2};
enum MatState {IDLE_NAV, MANUAL, RECOVERY};
enum SigJoy {UP, DOWN};



class RelayControlNode {
 public:
  RelayControlNode(ros::NodeHandle nh, const char* port, int baudrate);
  void cmd_vel_callback(const geometry_msgs::Twist& msg);
  void is_joy_actif(const actionlib_msgs::GoalStatusArray& msg);
  void is_recovery_actif(const move_base_msgs::RecoveryStatus& msg); //TODO: change the type of the message
  void timer_callback(const ros::TimerEvent& event);
  void shutdown();
  float compute_mean(float tab[]);
  void init_buffer();
  bool init_successful() { return init_success; }

 private:
  ros::Subscriber sub_cmd_vel, sub_joystick, sub_recovery;
  ros::Timer timer;

  // Déclaration du buffer circulaire pour les vitesses linéaires et angulaires
  float linear_vel_buffer[BUFFER_SIZE];
  float angular_vel_buffer[BUFFER_SIZE];
  int current_index;
  ros::Time timerStart;

  int serial;
  ros::Time timerStart = ros::Time::now();
  SpotState spot_state = ETEINT;
  BlinkState blink_state = B_ETEINT;
  MatState mat_state = IDLE_NAV;
  SigJoy sig_joy = DOWN;

  bool init_success;
  bool joy_actif = false;
  bool recovery_actif = false;
};

#endif  // ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

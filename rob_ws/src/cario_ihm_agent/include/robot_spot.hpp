// Copyright 2024 cario360

#ifndef ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_
#define ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mutex>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/RecoveryStatus.h"
#include "multi_obstacles_tracker_msgs/CameraDetectionStampedArray.h"
#include "multi_obstacles_tracker_msgs/CameraDetectionStamped.h"
#include "ros/ros.h"
#include <signal.h>

extern int serial;

#define PORT "/dev/ttyACM0"
#define BAUDRATE B9600

#define CMD_VEL_TOPIC "/mobile_base_controller/odom"
#define JOY_TOPIC "/joy_priority_action/status"
#define RECOVERY_TOPIC "/move_base/recovery_status"
#define CAMERA_TOPIC "/camera_detection_1"

#define SPOT_T_ON 0.25
#define SPOT_T_OFF 0.25
#define BUFFER_SIZE 10
#define HALF_FOV_DEG 15

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
  void cmd_vel_callback(const nav_msgs::Odometry& odom);
  void joy_callback(const actionlib_msgs::GoalStatusArray& msg);
  void recovery_callback(const move_base_msgs::RecoveryStatus& msg); //TODO: change the type of the message
  // void timer_callback(const ros::TimerEvent& event);
  void camera_detection_callback(const multi_obstacles_tracker_msgs::CameraDetectionStampedArray& msg);
  void shutdown();
  float compute_mean(float tab[]);
  void init_buffer();
  bool init_successful() { return init_success; }

 private:
  ros::Subscriber sub_cmd_vel, sub_joystick, sub_recovery, sub_camera_detection;
  // ros::Timer timer;

  // HARDWARE
  int serial;
  bool init_success;

  // Déclaration du buffer circulaire pour les vitesses linéaires et angulaires (blue spot)
  float linear_vel_buffer[BUFFER_SIZE];
  float angular_vel_buffer[BUFFER_SIZE];
  int current_index;
  double timerStart;
  std::mutex detection_mutex;
  bool human_in_fov;
  SpotState spot_state = ETEINT;
  BlinkState blink_state = B_ETEINT;
  
  // gyrophare
  MatState mat_state = IDLE_NAV;
  SigJoy sig_joy = DOWN;
  ros::Time latest_stamp;
  std::mutex recovery_mutex;
  bool joy_actif = false;
  bool recovery_actif = false;
};

#endif  // ROB_WS_SRC_CARIO_IHM_AGENT_INCLUDE_RELAY_CONTROLLER_NODE_HPP_

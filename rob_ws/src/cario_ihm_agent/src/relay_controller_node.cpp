// Copyright 2024 cario360

#include "relay_controller_node.hpp"

std::unique_ptr<RelayControlNode> relay_controller;

RelayControlNode::RelayControlNode(ros::NodeHandle nh, const char* port, int baudrate) {
  // Initialize serial port
  this->serial = open(port, O_RDWR | O_NOCTTY);

  if (this->serial == -1) {
    std::string error_message =
        "Error opening serial port '" + std::string(port) + "'";
    ROS_ERROR(error_message.c_str());
    this->init_success = false;
    return;
  }

  // Set serial port options
  struct termios options;
  tcgetattr(serial, &options);
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  tcsetattr(serial, TCSANOW, &options);

  std::string success_message =
      "Serial port '" + std::string(port) + "' opened successfully";
  ROS_INFO(success_message.c_str());
  write(this->serial, MAT_ORANGE_ON, 1);

  // Initialize ROS elements
  this->sub_cmd_vel = nh.subscribe(CMD_VEL_TOPIC, 1000,
                                  &RelayControlNode::cmd_vel_callback, this);
  this->sub_joystick = nh.subscribe(JOY_TOPIC, 1000,
                                  &RelayControlNode::is_joy_actif, this);
  this->sub_recovery = nh.subscribe(RECOVERY_TOPIC, 1000, 
                                  &RelayControlNode::is_recovery_actif, this);
  this->sub_human_detection = nh.subscribe(HUMAN_DETECTION_TOPIC, 1000,
                                  &RelayControlNode::is_human_detected, this);         
  this->timer = nh.createTimer(ros::Duration(0.2),
                                      &RelayControlNode::timer_callback, this);
  this->init_success = true;
}

void RelayControlNode::shutdown()
{
  write(this->serial, ALL_OFF, 1);
  close(this->serial);
}

void RelayControlNode::cmd_vel_callback(const geometry_msgs::Twist& msg) {
  bool speed_null = msg.linear.x == 0.0 && msg.angular.z == 0.0;

  switch (spot_state) {
    case STOPPED:
      if (!speed_null) {
        ROS_INFO("Robot moving, spot turns ON");
        write(this->serial, SPOT_ON, 1);
        spot_state = MOUVEMENT;
      }
      break;

    case MOUVEMENT:
      while (!speed_null) {
        if (person_detected) {
          ROS_INFO("Person detected, spot BLINKS");
          ros::Duration passed = ros::Time::now() - this->timerStart;
          if ((passed >= ros::Duration(0.0) && passed < ros::Duration(0.25)) ||
              (passed >= ros::Duration(0.5) && passed < ros::Duration(0.75))) {
            write(this->serial, SPOT_OFF, 1);
          } else if ((passed >= ros::Duration(0.25) && passed < ros::Duration(0.5)) ||
                     (passed >= ros::Duration(0.75) && passed < ros::Duration(1.0))) {
            write(this->serial, SPOT_ON, 1);
          }
        }
      }
      if (speed_null) {
        ROS_INFO("Robot stopped, starting timer");
        // start 5 seconde timer without pausing thread
        this->timerStart = ros::Time::now();
        spot_state = STOP_WAIT;
      }
      
      break;

    case STOP_WAIT:
      ros::Duration passed = ros::Time::now() - this->timerStart;

      if (!speed_null) {
        ROS_INFO("Robot moving, timer cancelled");
        spot_state = MOUVEMENT;
      } else if ((passed >= ros::Duration(STOP_DURATION - 1.0) &&
                  passed < ros::Duration(STOP_DURATION - 0.75)) ||
                 (passed >= ros::Duration(STOP_DURATION - 0.5) &&
                  passed < ros::Duration(STOP_DURATION - 0.25))) {
        write(this->serial, SPOT_OFF, 1);
      } else if ((passed >= ros::Duration(STOP_DURATION - 0.75) &&
                  passed < ros::Duration(STOP_DURATION - 0.5)) ||
                 (passed >= ros::Duration(STOP_DURATION - 0.25) &&
                  passed < ros::Duration(STOP_DURATION))) {
        write(this->serial, SPOT_ON, 1);
      } else if (passed >= ros::Duration(STOP_DURATION)) {
        write(this->serial, SPOT_OFF, 1);
        spot_state = STOPPED;
      }
      break;
  }
}

void RelayControlNode::timer_callback(const ros::TimerEvent& event) {
  switch (mat_state) {
    case IDLE_NAV:
      if (joy_actif) {
        ROS_INFO("Joystick active, ORANGE mat turns ON");
        write(this->serial, MAT_GREEN_OFF, 1);
        write(this->serial, MAT_ORANGE_ON, 1);
        mat_state = MANUAL;
      } else if (recovery_actif) {
        ROS_INFO("Recovery active, RED mat turns ON");
        write(this->serial, MAT_GREEN_OFF, 1);
        write(this->serial, MAT_RED_ON, 1);
        mat_state = RECOVERY;
      }
      break;
    
    case MANUAL:
      if (!joy_actif && !recovery_actif) {
        ROS_INFO("Joystick inactive, ORANGE mat turns OFF");
        write(this->serial, MAT_ORANGE_OFF, 1);
        write(this->serial, MAT_GREEN_ON, 1);
        mat_state = IDLE_NAV;
      } else if (!joy_actif && recovery_actif) {
        ROS_INFO("Recovery active, RED mat turns ON");
        write(this->serial, MAT_ORANGE_OFF, 1);
        write(this->serial, MAT_RED_ON, 1);
        mat_state = RECOVERY;
      }
      break;

    case RECOVERY:
      if (!recovery_actif) {
        ROS_INFO("Recovery inactive, RED mat turns OFF");
        write(this->serial, MAT_RED_OFF, 1);
        write(this->serial, MAT_GREEN_ON, 1);
        mat_state = IDLE_NAV;
      } else if (joy_actif) {
        ROS_INFO("Joystick active, ORANGE mat turns ON");
        write(this->serial, MAT_RED_OFF, 1);
        write(this->serial, MAT_ORANGE_ON, 1);
        mat_state = MANUAL;
      }
      break;
  }
}

void RelayControlNode::is_joy_actif(const sensor_msgs::Joy& msg) {
  if (msg.buttons[0] == 1 && sig_joy == DOWN) {
    sig_joy = UP;
    joy_actif = !joy_actif;
  } else if (msg.buttons[0] == 0 && sig_joy == UP) {
    sig_joy = DOWN;
  }
  return;
}

void RelayControlNode::is_recovery_actif(const move_base_msgs::RecoveryStatus& msg) //TODO: change to recoverymessage
{
  if (msg.current_recovery_number >= 0) {
    recovery_actif = true;
    }
  else {
    recovery_actif = false;
  }
}

void RelayControlNode::is_human_detected(const multi_obstacles_tracker_msgs::CameraDetectionStampedArray& msg) {
  for (int i = 0; i < msg.detections.size(); i++) {
    if (msg.detections[i].classification == "person" && (msg.detections[i].coordinates[1] >= 30.0 && msg.detections[i].coordinates[1] <= 330.0) && msg.detections[i].coordinates[0] <= 2.0) {
      person_detected = true;
      return;
    }
  }
  person_detected = false;
}

void sigint_handler(int sig) {
  relay_controller->shutdown();
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "relay_controller", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  relay_controller = std::make_unique<RelayControlNode>(nh, PORT, BAUDRATE);

  if (relay_controller->init_successful()) {
    signal(SIGINT, sigint_handler);
    ros::spin();
  }
  
  return 0;
}

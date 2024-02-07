// Copyright 2024 cario360

#include "relay_controller_node.hpp"

RelayControlNode::RelayControlNode(const char* port, int baudrate) {
  // Initialize serial port
  this->serial = open(port, O_RDWR | O_NOCTTY);

  if (this->serial == -1) {
    std::string error_message =
        "Error opening serial port '" + std::string(port) + "'";
    ROS_ERROR(error_message.c_str());
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

  // Initialize ROS elements
  this->sub_cmd_vel = this->nh.subscribe(CMD_VEL_TOPIC, 1000,
                                  &RelayControlNode::cmd_vel_callback, this);
  this->sub_joystick = this->nh.subscribe(JOY_TOPIC, 1000,
                                  &RelayControlNode::is_joy_actif, this);
  this->sub_recovery = this->nh.subscribe(RECOVERY_TOPIC, 1000, 
                                  &RelayControlNode::is_recovery_actif, this);
  this->timer = this->nh.createTimer(ros::Duration(0.2),
                                      &RelayControlNode::timer_callback, this);
  ros::spin();
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

void RelayControlNode::is_joy_actif(const actionlib_msgs::GoalStatusArray& msg) {
  if (sig_joy == DOWN && msg.status_list.size() > 0) {
    sig_joy = UP;
    joy_actif = !joy_actif;
  } else if (sig_joy == UP && msg.status_list.size() == 0) {
    sig_joy = DOWN;
  }
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "relay_controller");
  RelayControlNode relay_controller(PORT, BAUDRATE);

  return 0;
}

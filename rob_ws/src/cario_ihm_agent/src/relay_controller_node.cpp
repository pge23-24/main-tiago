// Copyright 2024 cario360

#include "include/relay_controller_node.hpp"

RelayControlNode::RelayControlNode(const char* port, int baudrate) {
  // Initialize serial port
  this->serial = open(port, O_RDWR | O_NOCTTY);

  if (this->serial == -1) {
    std::string error_message =
        "Error opening serial port '" + std::string(port) + "'";
    ROS_ERROR(error_message.c_str());
  } else {
    struct termios options;
    tcgetattr(serial, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    tcsetattr(serial, TCSANOW, &options);

    std::string success_message =
        "Serial port '" + std::string(port) + "' opened successfully";
    ROS_INFO(success_message.c_str());

    this->sub = this->nh.subscribe(CMD_VEL_TOPIC, 1000,
                                   &RelayControlNode::cmd_vel_callback, this);
    ros::spin();
  }
}

void RelayControlNode::cmd_vel_callback(const geometry_msgs::Twist& msg) {
  bool speed_null = msg.linear.x == 0.0 && msg.angular.z == 0.0;

  switch (state) {
    case STATE_STOPPED:
      if (!speed_null) {
        ROS_INFO("Robot moving, spot turns ON");
        write(this->serial, LED_ON, 1);
        state = STATE_MOUVEMENT;
      }
      break;

    case STATE_MOUVEMENT:
      if (speed_null) {
        ROS_INFO("Robot stopped, starting timer");
        // start 5 seconde timer without pausing thread
        this->timerStart = ros::Time::now();
        state = STATE_STOP_WAIT;
      }
      break;

    case STATE_STOP_WAIT:
      ros::Duration passed = ros::Time::now() - this->timerStart;

      if (!speed_null) {
        ROS_INFO("Robot moving, timer cancelled");
        state = STATE_MOUVEMENT;
      } else if ((passed >= ros::Duration(STOP_DURATION - 1.0) &&
                  passed < ros::Duration(STOP_DURATION - 0.75)) ||
                 (passed >= ros::Duration(STOP_DURATION - 0.5) &&
                  passed < ros::Duration(STOP_DURATION - 0.25))) {
        write(this->serial, LED_OFF, 1);
      } else if ((passed >= ros::Duration(STOP_DURATION - 0.75) &&
                  passed < ros::Duration(STOP_DURATION - 0.5)) ||
                 (passed >= ros::Duration(STOP_DURATION - 0.25) &&
                  passed < ros::Duration(STOP_DURATION))) {
        write(this->serial, LED_ON, 1);
      } else if (passed >= ros::Duration(STOP_DURATION)) {
        write(this->serial, LED_OFF, 1);
        state = STATE_STOPPED;
      }
      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "relay_controller");
  RelayControlNode relay_controller(PORT, BAUDRATE);

  return 0;
}

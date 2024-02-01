#ifndef SPOT_CONTROL_NODE_HPP
#define SPOT_CONTROL_NODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

extern int serial;

void cmd_vel_callback(const geometry_msgs::Twist& msg);

#endif // SPOT_CONTROL_NODE_HPP
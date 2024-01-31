/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */
#ifndef ROB_WS_SRC_CARIO_TOOLS_INCLUDE_GAZEBO_SYNCHRONIZER_HPP_
#define ROB_WS_SRC_CARIO_TOOLS_INCLUDE_GAZEBO_SYNCHRONIZER_HPP_

#include <cario_tools/ModelStatesTimeStamped.h>
#include <cario_tools/TFTimeStamped.h>

void callback(const gazebo_msgs::ModelStatesConstPtr& msg);

void callback2(const tf::tfMessageConstPtr& msg);

#endif  // ROB_WS_SRC_CARIO_TOOLS_INCLUDE_GAZEBO_SYNCHRONIZER_HPP_

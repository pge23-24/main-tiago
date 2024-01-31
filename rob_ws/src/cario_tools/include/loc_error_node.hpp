/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */
#ifndef ROB_WS_SRC_CARIO_TOOLS_INCLUDE_LOC_ERROR_NODE_HPP_
#define ROB_WS_SRC_CARIO_TOOLS_INCLUDE_LOC_ERROR_NODE_HPP_

#include <cario_tools/ModelStatesTimeStamped.h>
#include <cario_tools/TFTimeStamped.h>

const char tf_name[] = "base_link";
const char gazebo_name[] = "pmb2";

struct Data {
  double x;
  double y;
  double theta;
};

Data get_loc_error(Data gazebo_data, Data tf_data);

Data get_gazebo_loc(
    const cario_tools::ModelStatesTimeStampedConstPtr& ground_truth_msg);
Data get_tf_loc(const cario_tools::TFTimeStampedConstPtr& slam_loc_msg);

void callback(
    const cario_tools::ModelStatesTimeStampedConstPtr& ground_truth_msg,
    const cario_tools::TFTimeStampedConstPtr& slam_loc_msg);

#endif  // ROB_WS_SRC_CARIO_TOOLS_INCLUDE_LOC_ERROR_NODE_HPP_

/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */
#include "../include/loc_error_node.hpp"

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

// TF
#include <tf/transform_listener.h>

// GAZEBO
#include <cario_tools/ModelStatesTimeStamped.h>
#include <gazebo_msgs/ModelStates.h>

// Autres
#include <std_msgs/String.h>

#include <cmath>
#include <iomanip>

// namespaces
using message_filters::Subscriber;
using tf::TransformListener;

void callback(
    const cario_tools::ModelStatesTimeStampedConstPtr& ground_truth_msg,
    const tf::tfMessageConstPtr& slam_loc_msg) {
  // GET GAZEBO DATA
  Data gazebo_data = get_gazebo_loc(ground_truth_msg);

  Data tf_data = get_tf_loc(slam_loc_msg);

  // GET ERROR
  Data error = get_loc_error(gazebo_data, tf_data);
  ROS_INFO_STREAM("Localization error: "
                  << "x: " << std::setw(12) << std::left << std::fixed
                  << error.x << "y: " << std::setw(12) << std::left
                  << std::fixed << error.y << "theta: " << std::setw(12)
                  << std::left << std::fixed << error.theta);
}

Data get_loc_error(Data gazebo_data, Data tf_data) {
  Data error;
  error.x = std::abs(gazebo_data.x - tf_data.x);
  error.y = std::abs(gazebo_data.y - tf_data.y);
  error.theta = std::abs(gazebo_data.theta - tf_data.theta);

  return error;
}

Data get_gazebo_loc(
    const cario_tools::ModelStatesTimeStampedConstPtr& ground_truth_msg) {
  // GET GAZEBO DATA
  Data gazebo_data;
  gazebo_msgs::ModelStates model_states = ground_truth_msg->model_states;

  if (!model_states.name.empty()) {
    int id = 0;
    for (int i = 0; i < model_states.name.size(); i++) {
      if (model_states.name.at(i) == gazebo_name) {
        id = i;
        break;
      }
    }
    gazebo_data.x = model_states.pose.at(id).position.x;
    gazebo_data.y = model_states.pose.at(id).position.y;
    gazebo_data.theta = tf::getYaw(model_states.pose.at(id).orientation);
  }
  return gazebo_data;
}

Data get_tf_loc(const tf::tfMessageConstPtr& slam_loc_msg) {
  // GET TF DATA
  Data tf_data;
  if (!slam_loc_msg->transforms.empty()) {
    int id = 0;
    for (int i = 0; i < slam_loc_msg->transforms.size(); i++) {
      if (slam_loc_msg->transforms.at(i).child_frame_id == tf_name) {
        id = i;
        break;
      }
    }
    tf_data.x = slam_loc_msg->transforms.at(id).transform.translation.x;
    tf_data.y = slam_loc_msg->transforms.at(id).transform.translation.y;
    tf_data.theta =
        tf::getYaw(slam_loc_msg->transforms.at(id).transform.rotation);
  }
  return tf_data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "loc_error_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<cario_tools::ModelStatesTimeStamped>
      ground_truth_sub(nh, "/cario/simulation/model_states_time_stamped", 1);
  message_filters::Subscriber<tf::tfMessage> slam_loc_sub(nh, "/tf", 1);

  message_filters::TimeSynchronizer<cario_tools::ModelStatesTimeStamped,
                                    tf::tfMessage>
      sync(ground_truth_sub, slam_loc_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

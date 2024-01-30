/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */

#include <gazebo_msgs/ModelStates.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <iomanip>
#include <queue>

using message_filters::Subscriber;
using tf::TransformListener;

const char tf_name[] = "base_link";
const char gazebo_name[] = "pmb2";
struct Data {
  double x;
  double y;
  double theta;
};

std::queue<Data> gazebo_fifo;
std::queue<Data> tf_fifo;

Data get_loc_error(Data gazebo_data, Data tf_data) {
  Data error;
  error.x = std::abs(gazebo_data.x - tf_data.x);
  error.y = std::abs(gazebo_data.y - tf_data.y);
  error.theta = std::abs(gazebo_data.theta - tf_data.theta);

  return error;
}

void callback(const gazebo_msgs::ModelStatesConstPtr& ground_truth_msg,
              const tf::tfMessageConstPtr& slam_loc_msg) {
  // GET GAZEBO DATA
  Data gazebo_data;
  if (!ground_truth_msg->name.empty()) {
    int id = 0;
    for (int i = 0; i < ground_truth_msg->name.size(); i++) {
      if (ground_truth_msg->name.at(i) == gazebo_name) {
        id = i;
        break;
      }
    }
    gazebo_data.x = ground_truth_msg->pose.at(id).position.x;
    gazebo_data.y = ground_truth_msg->pose.at(id).position.y;
    gazebo_data.theta = tf::getYaw(ground_truth_msg->pose.at(id).orientation);
  }

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
  // GET ERROR
  if (!gazebo_data.x || !tf_data.x || !gazebo_data.y || !tf_data.y ||
      !gazebo_data.theta || !tf_data.theta) {
    ROS_INFO_STREAM("One of the data is empty");
  } else {
    Data error = get_loc_error(gazebo_data, tf_data);
    ROS_INFO_STREAM("Localization error: "
                    << "x: " << std::setw(12) << std::left << std::fixed
                    << error.x << "y: " << std::setw(12) << std::left
                    << std::fixed << error.y << "theta: " << std::setw(12)
                    << std::left << std::fixed << error.theta);
  }
}

int main(int argc, char** argv) {
  std::queue<int> fifo;
  ros::init(argc, argv, "loc_error_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<gazebo_msgs::ModelStates> ground_truth_sub(
      nh, "/gazebo/model_states", 10);
  message_filters::Subscriber<tf::tfMessage> slam_loc_sub(nh, "/tf", 10);

  typedef message_filters::sync_policies::ApproximateTime<
      gazebo_msgs::ModelStates, tf::tfMessage>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), ground_truth_sub, slam_loc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

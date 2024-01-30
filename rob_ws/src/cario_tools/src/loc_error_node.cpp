/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 *
 *
 *
 *
 *
 */

#include <gazebo_msgs/ModelStates.h>
#include <message_filters/subscriber.h>
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

/*
void callback(const gazebo_msgs::ModelStates::ConstPtr&
msg,tf::tfMessage::ConstPtr& msg2)
{
    // ROS_INFO("Received message from topics: %s", msg->name.at(0).c_str());
    // ROS_INFO("Received message from topics: %s",
msg2->transforms[0].child_frame_id.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loc_error_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<gazebo_msgs::ModelStates> ground_truth_sub(nh,
"/gazebo_msgs/model_states", 1); message_filters::Subscriber<tf::tfMessage>
slam_loc_sub(nh, "/tf", 1);

    TimeSynchronizer<gazebo_msgs::ModelStates, tf::tfMessage>
sync(ground_truth_sub, slam_loc_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
*/

void callback1(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  if (!msg->name.empty()) {
    Data data;
    int id = 0;

    for (int i = 0; i < msg->name.size(); i++) {
      if (msg->name.at(i) == gazebo_name) {
        id = i;
        break;
      }
    }
    data.x = msg->pose.at(id).position.x;
    data.y = msg->pose.at(id).position.y;
    data.theta = tf::getYaw(msg->pose.at(id).orientation);

    gazebo_fifo.push(data);
  }
}

void callback2(const tf::tfMessage::ConstPtr& msg) {
  Data data;
  int id = 0;

  for (int i = 0; i < msg->transforms.size(); i++) {
    if (msg->transforms.at(i).child_frame_id == tf_name) {
      id = i;
      break;
    }
  }
  data.x = msg->transforms.at(id).transform.translation.x;
  data.y = msg->transforms.at(id).transform.translation.y;
  data.theta = tf::getYaw(msg->transforms.at(id).transform.rotation);

  tf_fifo.push(data);
}

Data get_loc_error(Data gazebo_data, Data tf_data) {
  Data error;
  error.x = std::abs(gazebo_data.x - tf_data.x);
  error.y = std::abs(gazebo_data.y - tf_data.y);
  error.theta = std::abs(gazebo_data.theta - tf_data.theta);

  return error;
}

int main(int argc, char** argv) {
  std::queue<int> fifo;
  ros::init(argc, argv, "loc_error_node");
  ros::NodeHandle nh;

  ros::Subscriber ground_truth_sub =
      nh.subscribe("/gazebo/model_states", 10, callback1);
  ros::Subscriber slam_loc_sub = nh.subscribe("/tf", 10, callback2);

  while (ros::ok()) {
    ros::spinOnce();

    while (!gazebo_fifo.empty() && !tf_fifo.empty()) {
      Data gazebo_data = gazebo_fifo.front();
      Data tf_data = tf_fifo.front();
      Data error = get_loc_error(gazebo_data, tf_data);

      ROS_INFO_STREAM("Localization error: "
                      << "x: " << std::setw(12) << std::left << std::fixed
                      << error.x << "y: " << std::setw(12) << std::left
                      << std::fixed << error.y << "theta: " << std::setw(12)
                      << std::left << std::fixed << error.theta);

      gazebo_fifo.pop();
      tf_fifo.pop();
    }
  }

  return 0;
}

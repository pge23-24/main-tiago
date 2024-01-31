/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */

#include <cario_tools/ModelStatesTimeStamped.h>
#include <cario_tools/TFTimeStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>

ros::Publisher pub;

void callback(const gazebo_msgs::ModelStatesConstPtr& msg) {
  cario_tools::ModelStatesTimeStamped msgSync;
  msgSync.header.stamp = ros::Time::now();
  msgSync.model_states = *msg;
  pub.publish(msgSync);
}

void callback2(const tf::tfMessageConstPtr& msg) {
  cario_tools::TFTimeStamped msgSync;
  msgSync.header.stamp = ros::Time::now();
  msgSync.transforms = *msg;
  pub.publish(msgSync);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_synchronizer_node");
  ros::NodeHandle nh;

  ROS_INFO("gazebo_synchronizer_node started");

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, callback);
  pub = nh.advertise<cario_tools::ModelStatesTimeStamped>(
      "/cario/simulation/model_states_time_stamped", 10);

  ros::Subscriber sub2 = nh.subscribe("/tf", 10, callback2);
  pub = nh.advertise<cario_tools::TFTimeStamped>(
      "/cario/simulation/tf_time_stamped", 10);

  ros::spin();

  return 0;
}

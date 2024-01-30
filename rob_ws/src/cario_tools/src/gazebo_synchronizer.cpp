/**
 * Calculate the localization error between gazebo and slam
 * Copyright 2024 Arthur Vigouroux
 */

#include <cario_tools/ModelStatesTimeStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>

ros::Publisher pub;

void callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  cario_tools::ModelStatesTimeStamped
      msgSync;  // Replace with the actual name of your package and message
  msgSync.header.stamp = ros::Time::now();
  msgSync.model_states = *msg;
  pub.publish(msgSync);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, callback);
  pub = nh.advertise<cario_tools::ModelStatesTimeStamped>(
      "/cario/simulation/model_states_time_stamped",
      10);  // Replace with the actual name of your package and message

  ros::spin();

  return 0;
}

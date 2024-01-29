#ifndef CLASSICAL
#define CLASSICAL
#include <memory>
#include <iostream>
#include <fstream>
#endif

#ifndef ROS
#define ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#endif

#ifndef CV_BRIDGE
#define CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#endif

class CameraYolo : public rclcpp::Node
{
public:
  CameraYolo(int carmera_id = 1)
  : Node("minimal_subscriber")
  {

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "Cam" + std::to_string(camera_id) + "/image_raw", 10, std::bind(&CameraYolo::topic_callback, this, _1));
    this->camera_id = camera_id;
    this->topic_name = "annotated_images_" + std::to_string(camera_id)
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const;


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  YOLOSettings config;
  YOLOv5 detector(config);
  cv::Mat cv_image;
  int camera_id;
  std::string topic_name;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
};
#ifndef CAMERAYOLO_HPP_
#define CAMERAYOLO_HPP_

#ifndef CLASSICAL
#define CLASSICAL
#include <memory>
#include <iostream>
#include <fstream>
#endif

#ifndef ROS
#define ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#endif

#ifndef CV_BRIDGE
#define CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#endif

#include "YOLOv5.hpp"

using std::placeholders::_1;

class CameraYolo : public rclcpp::Node
{
public:
  CameraYolo(int camera_id = 1);
  
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const;


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  //YOLOSettings config;

  int camera_id;
  std::string topic_name;
  YOLOv5* detector;
};

#endif // CAMERAYOLO_HPP_
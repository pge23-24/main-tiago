#ifndef CLASSICAL
#define CLASSICAL
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
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

#include "../include/YOLOv5.hpp"
#include "../include/CameraYolo.hpp"

using std::placeholders::_1;

CameraYolo::CameraYolo(int camera_id) : Node("CameraYolo")
  {
    RCLCPP_INFO(this->get_logger(), "Starting setup for camera %d finished", camera_id);

    this->camera_id = camera_id;
    std::string sub_topic = "Cam" + std::to_string(camera_id) + "/image_raw";
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(sub_topic, 10, std::bind(&CameraYolo::topic_callback, this, _1));
    this->topic_name = "annotated_images_" + std::to_string(camera_id);
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    this->detector = YOLOv5::getInstance();


    RCLCPP_INFO(this->get_logger(), "Setup for camera %d finished", this->camera_id);

  }

void CameraYolo::topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const
{
  
  RCLCPP_INFO(this->get_logger(), "Image received from camera %d", this->camera_id);

  auto t_start = std::chrono::high_resolution_clock::now();
  

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  cv::Mat cv_image = cv_ptr->image;

  this->detector->findObjects(cv_image);

  std_msgs::msg::Header header;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_image);

  sensor_msgs::msg::Image img_msg;
  img_bridge.toImageMsg(img_msg);

  publisher_->publish(img_msg);


  auto t_end = std::chrono::high_resolution_clock::now();
  
  double time_taken = std::chrono::duration<double, std::milli>(t_end-t_start).count();
  RCLCPP_INFO(this->get_logger(), "Time taken =  %f", time_taken);
}



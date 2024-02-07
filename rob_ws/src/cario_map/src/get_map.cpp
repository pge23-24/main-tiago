// Copyright 2024 cario

#include <fstream>
#include <iostream>
#include <vector>

#include "cario_map/GetMapInfo.h"
#include "include/GraymapImage.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "yaml-cpp/yaml.h"

class MapServiceNode {
 public:
  MapServiceNode() {
    // Advertise the service
    service_ = nh_.advertiseService("get_map",
                                    &MapServiceNode::GetMapInfoCallback, this);
    pkg_root = ros::package::getPath("cario_map");

    ROS_INFO("Map service ready !");
  }

  bool GetMapInfoCallback(const cario_map::GetMapInfo::Request& req,
                          cario_map::GetMapInfo::Response* res) {
    // Path to your local .pgm file
    std::string yamlFilePath = pkg_root + "/maps/workshop/usine_v2.yaml";
    std::string pgmFilePath = pkg_root + "/maps/workshop/usine_v2.pgm";

    // Read the YAML config file
    YAML::Node config = YAML::LoadFile(yamlFilePath);

    // Extract metadata from the YAML file
    res.resolution = config["resolution"].as<double>();
    res.origin = config["origin"].as<std::vector<double>>();
    res.occupied_thresh = config["occupied_thresh"].as<double>();
    res.free_thresh = config["free_thresh"].as<double>();

    // Read the binary PGM file
    ImageUtils::GraymapImage image;

    if (!image.LoadImage(pgmFilePath)) {
      ROS_ERROR("Failed to load .pgm file");
      res.success = false;
      return false;
    }

    // Create an Image message
    res.success = true;
    res.map.width = image.GetWidth();
    res.map.height = image.GetHeight();
    res.map.encoding = "mono8";
    res.map.step = image.GetWidth();
    res.map.data.resize(image.GetWidth() * image.GetHeight());

    // Read image pixel data into the Image message
    for (unsigned int y = 0; y < image.GetHeight(); y++) {
      for (unsigned int x = 0; x < image.GetWidth(); x++) {
        res.map.data[y * image.GetWidth() + x] = image.GetPixel(x, y);
      }
    }

    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  std::string pkg_root;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_service_node");
  MapServiceNode mapServiceNode;
  ros::spin();
  return 0;
}

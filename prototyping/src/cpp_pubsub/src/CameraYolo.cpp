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

#include "YOLOv5.hpp"
#include "CameraYolo.hpp"

using std::placeholders::_1;



void CameraYolo::topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const
{
    RCLCPP_INFO(this->get_logger(), "Image received from camera %d", this->camera_id);

    cv::cvtColor(
        cv_bridge::toCvCopy(image,'passthrough'), 
        cv_image, 
        cv::COLOR_BGR2RGB);

    detector.findObjects(cv_image);

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_image);
    img_bridge.toImageMsg(img_msg);

    publisher_->publish(img_msg);

}


/*
int main(int argc, char **argv)
{
    YOLOSettings config;
    YOLOv5 detector(config);

    cv::Mat frame;
    cv::VideoCapture capture("../demo.mp4");
    if (!capture.isOpened())
    {
        std::cerr << "Failed to open the video file.\n";
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float fps = -1;

    while (true)
    {
        capture.read(frame);
        if (frame.empty())
        {
            std::cout << "End of video stream.\n";
            break;
        }

        std::vector<ObjectDetection> detectedObjects;
        detector.findObjects(frame);

        frame_count++;
        if (frame_count >= 30)
        {
            auto end = std::chrono::high_resolution_clock::now();
            fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            frame_count = 0;
            start = std::chrono::high_resolution_clock::now();
        }

        if (fps > 0)
        {
            std::ostringstream fpsText;
            fpsText << std::fixed << std::setprecision(2);
            fpsText << "FPS: " << fps;
            cv::putText(frame, fpsText.str(), cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Detection Results", frame);
        if (cv::waitKey(1) != -1)
        {
            capture.release();
            break;
        }
    }
    return 0;
}

*/





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraYolo>());
  rclcpp::shutdown();
  return 0;
}

/*
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
}
*/
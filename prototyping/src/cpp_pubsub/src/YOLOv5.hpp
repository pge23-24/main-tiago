#include <memory>
#include <iostream>
#include <fstream>

#ifndef OPENCV
#include <opencv2/opencv.hpp>
#endif


struct YOLOSettings
{
    const std::string label_path = "../data/classes.txt";
    const std::string onnx_path = "../data/yolov5s.onnx";
    const float Score_threshold = 0.2;
    const float NMS_threshold = 0.4;
    const float Conf_threshold = 0.4;
    const bool use_cuda = false;
};


//individuelle
struct ObjectDetection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

class YOLOv5
{
public:
    YOLOv5(YOLOSettings config) : label_path(config.label_path), onnx_path(config.onnx_path), Score_threshold(config.Score_threshold), NMS_threshold(config.NMS_threshold), Conf_threshold(config.Conf_threshold)
    {
        this->readClassLabels();
        this->setupNetwork(config.use_cuda);
    };

    void findObjects(cv::Mat &image);

private:
    const std::string label_path;
    const std::string onnx_path;
    const float Score_threshold;
    const float NMS_threshold;
    const float Conf_threshold;
    const float Input_width = 640;
    const float Input_height = 640;

    cv::dnn::Net net;
    std::vector<std::string> class_list;

    void setupNetwork(bool is_cuda);
    void readClassLabels();
    void annotateImage(cv::Mat &, std::vector<ObjectDetection> &);

    cv::Mat prepareImage(const cv::Mat &source);
};
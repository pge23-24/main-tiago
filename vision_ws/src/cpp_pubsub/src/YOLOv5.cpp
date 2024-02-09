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

#include "../include/YOLOv5.hpp"


using std::placeholders::_1;

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 200, 50), cv::Scalar(50, 200, 50), cv::Scalar(50, 200, 200), cv::Scalar(200, 50, 50)};

YOLOv5* YOLOv5::inst_ = NULL;

YOLOv5* YOLOv5::getInstance(){
    if (inst_ == NULL){
        YOLOSettings config;
        inst_ = new YOLOv5(config);
    }
    return(inst_);
}

void YOLOv5::readClassLabels()
{
    std::ifstream ifs(label_path);
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    std::cout << "Class labels loaded successfully, total classes:  " << class_list.size() << std::endl;
}

void YOLOv5::setupNetwork(bool is_cuda)
{

    std::cout << "CHECKER 1" << std::endl;
    net = cv::dnn::readNet(onnx_path);


    std::cout << "CHECKER 2" << std::endl;
    if (is_cuda)
    {
        std::cout << "Running on CUDA\n";
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        std::cout << "Running on CPU\n";
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat YOLOv5::prepareImage(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = std::max(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void YOLOv5::findObjects(cv::Mat &image)
{
    cv::Mat blob;
    auto input_image = prepareImage(image);
    cv::dnn::blobFromImage(input_image, blob, 1 / 255., cv::Size(Input_width, Input_height), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / Input_width;
    float y_factor = input_image.rows / Input_height;

    float *data = (float *)outputs[0].data;

    const int dimensions = 85;
    const int rows = 25200;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];
        if (confidence >= Conf_threshold)
        {
            float *classes_scores = data + 5;
            cv::Mat scores(1, class_list.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > Score_threshold)
            {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        data += dimensions;
    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, Score_threshold, NMS_threshold, nms_result);

    std::vector<ObjectDetection> output;
    for (int i = 0; i < static_cast<int>(nms_result.size()); i++)
    {
        int idx = nms_result[i];
        ObjectDetection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }

    this->annotateImage(image, output);
}

void YOLOv5::annotateImage(cv::Mat &image, std::vector<ObjectDetection> &output)
{
    for (const auto &detection : output)
    {
        const auto &box = detection.box;
        int classId = detection.class_id;
        float confidence = detection.confidence;

        cv::Scalar boxColor(0, 0, 255);

        cv::rectangle(image, box, boxColor, 2);

        std::string label = class_list[classId] + ": " + cv::format("%.2f", confidence);

        int fontFace = cv::FONT_HERSHEY_DUPLEX;
        double fontScale = 0.5;
        int thickness = 1;

        int baseline;
        cv::Size textSize = cv::getTextSize(label, fontFace, fontScale, thickness, &baseline);

        cv::rectangle(image, cv::Point(box.x, box.y - textSize.height - baseline),
                      cv::Point(box.x + textSize.width, box.y), boxColor, cv::FILLED);

        cv::putText(image, label, cv::Point(box.x, box.y - baseline), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
    }
}
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 200, 50), cv::Scalar(50, 200, 50), cv::Scalar(50, 200, 200), cv::Scalar(200, 50, 50)};


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
    net = cv::dnn::readNet(onnx_path);
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
    for (int i = 0; i < nms_result.size(); i++)
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




class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(int carmera_id)
  : Node("minimal_subscriber")
  {

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "Cam" + std::to_string(camera_id) + "/image_raw", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    this->camera_id = camera_id;
    this->topic_name = "annotated_images_" + std::to_string(camera_id)
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const
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


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  YOLOSettings config;
  YOLOv5 detector(config);
  cv::Mat cv_image;
  int camera_id;
  std::string topic_name;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
}
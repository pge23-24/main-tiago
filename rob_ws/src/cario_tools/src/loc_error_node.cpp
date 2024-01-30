#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <queue>

using namespace tf;
using namespace message_filters;

const std::string tf_name = "base_footprint";
struct Data {;
    double x;
    double y;
    double theta;
};

std::queue<Data> gazebo_fifo;
std::queue<Data> tf_fifo;


/*
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg,tf::tfMessage::ConstPtr& msg2)
{
    // ROS_INFO("Received message from topics: %s", msg->name.at(0).c_str());
    // ROS_INFO("Received message from topics: %s", msg2->transforms[0].child_frame_id.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loc_error_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<gazebo_msgs::ModelStates> ground_truth_sub(nh, "/gazebo_msgs/model_states", 1);
    message_filters::Subscriber<tf::tfMessage> slam_loc_sub(nh, "/tf", 1);

    TimeSynchronizer<gazebo_msgs::ModelStates, tf::tfMessage> sync(ground_truth_sub, slam_loc_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
*/

void callback1(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (!msg->name.empty()) {
        Data data;
        int id = 0;

        for (int i = 0; i < msg->name.size(); i++) {
            if (msg->name.at(i) == tf_name) {
                id = i;
                break;
            }
        }
        data.x = msg->pose.at(id).position.x;
        data.y = msg->pose.at(id).position.y;
        data.theta = tf::getYaw(msg->pose.at(id).orientation);

        gazebo_fifo.push(data);
    }
    
}

void callback2(const tf::tfMessage::ConstPtr& msg)
{
    Data data;
    int id = 0;

    for (int i = 0; i < msg->transforms.size(); i++) {
        if (msg->transforms.at(i).child_frame_id == tf_name) {
            id = i;
            break;
        }
    }
    data.x = msg->transforms.at(id).transform.translation.x;
    data.y = msg->transforms.at(id).transform.translation.y;
    data.theta = tf::getYaw(msg->transforms.at(id).transform.rotation);

    tf_fifo.push(data);
}

Data get_loc_error(Data gazebo_data, Data tf_data)
{
    Data error;
    error.x = gazebo_data.x - tf_data.x;
    error.y = gazebo_data.y - tf_data.y;
    error.theta = gazebo_data.theta - tf_data.theta;

    return error;
}

int main(int argc, char** argv)
{   
    std::queue<int> fifo;
    ros::init(argc, argv, "loc_error_node");
    ros::NodeHandle nh;

    ros::Subscriber ground_truth_sub = nh.subscribe("/gazebo_msgs/model_states", 1, callback1);
    ros::Subscriber slam_loc_sub = nh.subscribe("/tf", 1, callback2);


    ros::spin();
    while (1) {
        if (!gazebo_fifo.empty() && !tf_fifo.empty()) {
            Data gazebo_data = gazebo_fifo.front();
            Data tf_data = tf_fifo.front();
            Data error = get_loc_error(gazebo_data, tf_data);
            ROS_INFO("x: %f, y: %f, theta: %f", error.x, error.y, error.theta);
            gazebo_fifo.pop();
            tf_fifo.pop();
        }
    }

    return 0;
}


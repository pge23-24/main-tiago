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

struct GazeboData {
    std::string model_name;
    double x;
    double y;
};

struct TFData {
    std::string frame_id;
};

std::queue<GazeboData> gazebo_fifo;
std::queue<TFData> tf_fifo;


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
    ROS_INFO("Received message from topics: %s", msg->name.at(0).c_str());
    
}

void callback2(const tf::tfMessage::ConstPtr& msg)
{
    ROS_INFO("Received message from topics: %s", msg->transforms[0].child_frame_id.c_str());
}

int main(int argc, char** argv)
{   
    std::queue<int> fifo;
    ros::init(argc, argv, "loc_error_node");
    ros::NodeHandle nh;

    ros::Subscriber ground_truth_sub = nh.subscribe("/gazebo_msgs/model_states", 1, callback1);
    ros::Subscriber slam_loc_sub = nh.subscribe("/tf", 1, callback2);

    ros::spin();

    return 0;
}


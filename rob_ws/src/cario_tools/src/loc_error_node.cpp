#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/tf.h>

using namespace tf;
using namespace message_filters;

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message from topics: %s", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loc_error_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<TF> ground_truth_sub(nh, "image", 1);
    message_filters::Subscriber<TF> slam_loc_sub(nh, "camera_info", 1);
    TimeSynchronizer<TF, TF> sync(ground_truth_sub, slam_loc_sub, 100);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}

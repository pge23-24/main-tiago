#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "multi_obstacles_tracker_msgs/ClusterArray.h"

#include <sstream>


class ObstaclesClustering
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    ros::Publisher clusters_pub_;

    // subscriber variables
    ros::Subscriber cloud_sub_;
    

    ObstaclesClustering(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        clusters_pub_ = n_.advertise<sensor_msgs::PointCloud>("/cluster", 10);

        // subscribe to the /filtered_scan topic
        cloud_sub_ = n_.subscribe<multi_obstacles_tracker_msgs::ClusterArray>("/filtered_scan", 10, &ObstaclesClustering::filteredScanCallback, this);
    }

    void filteredScanCallback(const multi_obstacles_tracker_msgs::ClusterArray::ConstPtr& msg)
    {
        
    }
};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "obstacles_clustering");

    // create a ROS node handle
    ros::NodeHandle n;

    // create scan filter
    ObstaclesClustering oc(n);

    // loop ros
    ros::spin();

    return 0;
}

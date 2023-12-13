#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ClusterStamped.h"
#include "multi_obstacles_tracker_msgs/ClusterStampedArray.h"

#include "kalman_ros/kalman.hpp"

#include <sstream>
#include <list>


class ObstaclesTracker
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    // ros::Publisher clusters_pub_;

    // subscriber variables
    ros::Subscriber clusters_sub_;

    // kalman filter list
    std::list<KalmanFilter> filters_;
    

    ObstaclesTracker(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        // clusters_pub_ = n_.advertise<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10);

        // subscribe to the /scan_filtered topic
        clusters_sub_ = n_.subscribe<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10, &ObstaclesTracker::clusterCallback, this);
    }

    void clusterCallback(const multi_obstacles_tracker_msgs::ClusterStampedArray::ConstPtr& clusters_in)
    {
        // if there is not kalman filter active
        if(filters_.empty())
        {
            // init a new kalman filter for each cluster
            for(unsigned i = 0; i < clusters_in->clusters.size(); i++)
            {
                // get the i-th cluster
                multi_obstacles_tracker_msgs::ClusterStamped cluster = clusters_in->clusters[i];

                
            }
        }
    }
};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "obstacles_tracking");

    // create a ROS node handle
    ros::NodeHandle n;

    // create scan filter
    ObstaclesTracker ot(n);

    // loop ros
    ros::spin();

    return 0;
}

#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ClusterStamped.h"
#include "multi_obstacles_tracker_msgs/ClusterStampedArray.h"

#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"

#include <sstream>


class ClusterViz
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    ros::Publisher polygon_pub_;

    // subscriber variables
    ros::Subscriber cluster_sub_;
    

    ClusterViz(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        polygon_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/cluster_polygoned", 10);

        // subscribe to the /scan_filtered topic
        cluster_sub_ = n_.subscribe<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10, &ClusterViz::clusterArrayCallback, this);
    }

    void clusterArrayCallback(const multi_obstacles_tracker_msgs::ClusterStampedArray::ConstPtr& cluster_in)
    {
        // TODO: generalize this for all the clusters
        geometry_msgs::PolygonStamped polygon;
        
        for(unsigned i = 0; i < cluster_in->clusters.size(); i++)
        {
            
            multi_obstacles_tracker_msgs::ClusterStamped first_cluster = cluster_in->clusters[i];
            polygon.header = first_cluster.header;
            geometry_msgs::Point32 p_min_max, p_max_min;

            p_min_max.x = first_cluster.lower_right_corner.x;
            p_min_max.y = first_cluster.upper_left_corner.y;
            p_min_max.z = first_cluster.upper_left_corner.z;

            p_max_min.x = first_cluster.upper_left_corner.x;
            p_max_min.y = first_cluster.lower_right_corner.y;
            p_max_min.z = first_cluster.upper_left_corner.z;

            polygon.polygon.points.push_back(first_cluster.upper_left_corner);
            polygon.polygon.points.push_back(p_min_max);
            polygon.polygon.points.push_back(first_cluster.lower_right_corner);
            polygon.polygon.points.push_back(p_max_min);
            polygon.polygon.points.push_back(first_cluster.upper_left_corner);
        }

        polygon_pub_.publish(polygon);
    }
};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "cluster_viz");

    // create a ROS node handle
    ros::NodeHandle n;

    // create cluster viz
    ClusterViz oc(n);

    // loop ros
    ros::spin();

    return 0;
}

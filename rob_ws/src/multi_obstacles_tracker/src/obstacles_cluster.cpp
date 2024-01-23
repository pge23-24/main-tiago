#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ClusterStamped.h"
#include "multi_obstacles_tracker_msgs/ClusterStampedArray.h"

#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "pcl/common/common.h"

#include <sstream>


class ObstaclesCluster
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    ros::Publisher clusters_pub_;

    // subscriber variables
    ros::Subscriber cloud_sub_;
    

    ObstaclesCluster(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        clusters_pub_ = n_.advertise<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10);

        // subscribe to the /scan_filtered topic
        cloud_sub_ = n_.subscribe<sensor_msgs::PointCloud2>("/scan_filtered", 10, &ObstaclesCluster::filteredScanCallback, this);
    }

    void filteredScanCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
    {
        // Convert the sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        multi_obstacles_tracker_msgs::ClusterStampedArray clusters_msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_in, *cloud);

        // Apply Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.6); // Set the tolerance for cluster creation
        ec.setMinClusterSize(100);    // Set the minimum number of points a cluster should have
        ec.setMaxClusterSize(1000);   // Set the maximum number of points a cluster should have
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // for each cluster
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            // get cloud of the cluster points
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cluster->points.push_back(cloud->points[*pit]);
            
            pcl::PointXYZ barycenterPoint;
            pcl::PointXYZ upperLeftPoint;
            pcl::PointXYZ lowerRightPoint;

            // compute barycenter
            for (const auto& point : cluster->points)
            {
                barycenterPoint.x += point.x;
                barycenterPoint.y += point.y;
                barycenterPoint.z += point.z;
            }

            size_t clusterSize = cluster->points.size();
            if (clusterSize > 0)
            {
                barycenterPoint.x /= clusterSize;
                barycenterPoint.y /= clusterSize;
                barycenterPoint.z /= clusterSize;
            }

            // get min and max 3D point
            pcl::getMinMax3D(*cluster, upperLeftPoint, lowerRightPoint);

            // Create and fill the ClusterStamped msg
            multi_obstacles_tracker_msgs::ClusterStamped cluster_msg;
            cluster_msg.header = cloud_in->header;

            cluster_msg.barycenter.x = barycenterPoint.x;
            cluster_msg.barycenter.y = barycenterPoint.y;
            cluster_msg.barycenter.z = barycenterPoint.z;

            cluster_msg.upper_left_corner.x = upperLeftPoint.x;
            cluster_msg.upper_left_corner.y = upperLeftPoint.y;
            cluster_msg.upper_left_corner.z = upperLeftPoint.z;

            cluster_msg.lower_right_corner.x = lowerRightPoint.x;
            cluster_msg.lower_right_corner.y = lowerRightPoint.y;
            cluster_msg.lower_right_corner.z = lowerRightPoint.z;

            clusters_msg.clusters.push_back(cluster_msg);
        }

        if(clusters_msg.clusters.size() > 0)
            clusters_pub_.publish(clusters_msg);
    }
};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "obstacles_clustering");

    // create a ROS node handle
    ros::NodeHandle n;

    // create scan filter
    ObstaclesCluster oc(n);

    // loop ros
    ros::spin();

    return 0;
}

#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ObstacleMeasureStamped.h"
#include "multi_obstacles_tracker_msgs/ObstacleMeasureStampedArray.h"

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
        clusters_pub_ = n_.advertise<multi_obstacles_tracker_msgs::ObstacleMeasureStampedArray>("/measures", 10);

        // subscribe to the /scan_filtered topic
        cloud_sub_ = n_.subscribe<sensor_msgs::PointCloud2>("/scan_filtered", 10, &ObstaclesCluster::filteredScanCallback, this);
    }

    void filteredScanCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
    {
        // Convert the sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        multi_obstacles_tracker_msgs::ObstacleMeasureStampedArray measures_msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_in, *cloud);

        // Apply Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.6); // Set the tolerance for cluster creation
        ec.setMinClusterSize(50);    // Set the minimum number of points a cluster should have
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
            
            Eigen::Vector2f centroid;
            centroid *= 0;
            for(const auto& point : cluster->points)
            {
                centroid[0] += point.x;
                centroid[1] += point.y;
            }
            centroid /= cluster->points.size();

            Eigen::Matrix2f covariance;
            covariance *= 0;
            for(const auto& point : cluster->points)
            {
                Eigen::Vector2f e_point;
                e_point << point.x, point.y;
                covariance += (e_point - centroid)*(e_point - centroid).transpose();
            }
            covariance /= (cluster->points.size() - 1);

            // Create and fill the ObstacleMeasureStamped msg
            multi_obstacles_tracker_msgs::ObstacleMeasureStamped measure_msg;
            measure_msg.header = cloud_in->header;

            measure_msg.mean[0] = centroid[0];
            measure_msg.mean[1] = centroid[1];

            // fill cov matrix line by line
            measure_msg.covariance[0] = covariance(0, 0);
            measure_msg.covariance[1] = covariance(0, 1);
            measure_msg.covariance[2] = covariance(1, 0);
            measure_msg.covariance[3] = covariance(1, 1);

            measures_msg.measures.push_back(measure_msg);
        }

        if(measures_msg.measures.size() > 0)
            clusters_pub_.publish(measures_msg);
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

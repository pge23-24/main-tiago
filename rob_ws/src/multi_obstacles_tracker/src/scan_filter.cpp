#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#include <sstream>


class ScanFilter
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    ros::Publisher filtered_scan_;

    // subscriber variables
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector_;

    // static map variables
    nav_msgs::OccupancyGrid map_;
    

    ScanFilter(ros::NodeHandle n) : n_(n)
    {
        // get static map (map_server must be launched before)
        ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap service;
        if(client.call(service))
        {
            ROS_INFO("Static map acquired");
            map_ = service.response.map;
        }
        else
        {
            ROS_ERROR("Static map not acquired. Call map_server node with the specific static map before");
            exit(1);
        }

        // publish to the /filtered_scan
        filtered_scan_ = n_.advertise<sensor_msgs::PointCloud2>("/scan_filtered", 10);

        // subscribe to the /scan topic
        scan_sub_.subscribe(n_, "/scan", 10);
        scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, listener_, "map", 10);
        scan_filter_->registerCallback(boost::bind(&ScanFilter::scanCallback, this, _1));
        scan_filter_->setTolerance(ros::Duration(0.01));
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        sensor_msgs::PointCloud cloud;
        sensor_msgs::PointCloud filtered_cloud;
        sensor_msgs::PointCloud2 out_cloud;

        try
        {
            // converts scan to point of cloud in the /map frame
            projector_.transformLaserScanToPointCloud("map", *msg, cloud, listener_);
            filtered_cloud.header = cloud.header;
        }
        catch(tf::TransformException& e)
        {
            ROS_ERROR("%s", e.what());
        }

        for(unsigned i = 0; i < cloud.points.size(); i++)
        {
            // get the grid pos of the map for each point in the world frame
            unsigned static_map_x = (unsigned) ((cloud.points[i].x - map_.info.origin.position.x) / map_.info.resolution);
            unsigned static_map_y = (unsigned) ((cloud.points[i].y - map_.info.origin.position.y) / map_.info.resolution);

            // check if the coordinate is out of boundary
            if(static_map_x >= 0 && static_map_x < map_.info.width && static_map_y >= 0 && static_map_y < map_.info.height)
            {
                int map_intensity = map_.data[static_map_x + static_map_y * map_.info.width];
                if(map_intensity == 0)
                {
                    // the point is not on a known obstacle, add it to the output cloud points
                    filtered_cloud.points.push_back(cloud.points[i]);
                }
            }
        }

        // converts PointCloud to PointCloud2
        sensor_msgs::convertPointCloudToPointCloud2(filtered_cloud, out_cloud);

        // publish the filtered points
        filtered_scan_.publish(out_cloud);
    }
};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "scan_filter");

    // create a ROS node handle
    ros::NodeHandle n;

    // create scan filter
    ScanFilter sf(n);

    // loop ros
    ros::spin();

    return 0;
}

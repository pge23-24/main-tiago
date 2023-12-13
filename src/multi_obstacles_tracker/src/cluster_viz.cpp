#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ClusterStamped.h"
#include "multi_obstacles_tracker_msgs/ClusterStampedArray.h"

#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <sstream>


class ClusterViz
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    ros::Publisher markers_pub_;

    // subscriber variables
    ros::Subscriber cluster_sub_;
    

    ClusterViz(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        markers_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/cluster_markers", 10);

        // subscribe to the /scan_filtered topic
        cluster_sub_ = n_.subscribe<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10, &ClusterViz::clusterArrayCallback, this);
    }

    void clusterArrayCallback(const multi_obstacles_tracker_msgs::ClusterStampedArray::ConstPtr& cluster_in)
    {
        // marker array composed of a point list marker and a line list marker
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker points_marker;
        visualization_msgs::Marker lines_marker;

        // set the shape of the markers
        points_marker.type = visualization_msgs::Marker::POINTS;
        lines_marker.type = visualization_msgs::Marker::LINE_LIST;

        // set the ns and the id of the markers
        points_marker.ns = "multi_obstacles_tracker";
        points_marker.id = 0;
        lines_marker.ns = "multi_obstacles_tracker";
        lines_marker.id = 1;

        // set action to ADD of the markers
        points_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.action = visualization_msgs::Marker::ADD;

        // set the pose of the markers
        points_marker.pose.position.x = 0;
        points_marker.pose.position.y = 0;
        points_marker.pose.position.z = 0;
        points_marker.pose.orientation.x = 0;
        points_marker.pose.orientation.y = 0;
        points_marker.pose.orientation.z = 0;
        points_marker.pose.orientation.w = 1;

        lines_marker.pose.position.x = 0;
        lines_marker.pose.position.y = 0;
        lines_marker.pose.position.z = 0;
        lines_marker.pose.orientation.x = 0;
        lines_marker.pose.orientation.y = 0;
        lines_marker.pose.orientation.z = 0;
        lines_marker.pose.orientation.w = 1;

        // set the point size of the points marker
        points_marker.scale.x = 0.08;
        points_marker.scale.y = 0.08;

        // set the width of the lines marker
        lines_marker.scale.x = 0.05;

        // set the color of the points marker
        points_marker.color.r = 1.0f;
        points_marker.color.a = 1.0;

        // set the color of the lines marker
        lines_marker.color.g = 1.0f;
        lines_marker.color.a = 1.0;
        
        for(unsigned i = 0; i < cluster_in->clusters.size(); i++)
        {
            // get the i-th cluster
            multi_obstacles_tracker_msgs::ClusterStamped cluster = cluster_in->clusters[i];

            // set the header of the marker array using the first cluster
            if(i == 0)
            {
                points_marker.header = cluster.header;
                lines_marker.header = cluster.header;
            }
            
            // get the barycenter
            geometry_msgs::Point barycenter;
            barycenter.x = cluster.barycenter.x;
            barycenter.y = cluster.barycenter.y;
            barycenter.z = cluster.barycenter.z;

            // add the barycenter point
            points_marker.points.push_back(barycenter);

            // add 4x2 points for each bounding box
            geometry_msgs::Point p_min_min, p_min_max, p_max_max, p_max_min;

            p_min_min.x = cluster.lower_right_corner.x;
            p_min_min.y = cluster.lower_right_corner.y;
            p_min_min.z = cluster.lower_right_corner.z;

            p_min_max.x = cluster.lower_right_corner.x;
            p_min_max.y = cluster.upper_left_corner.y;
            p_min_max.z = cluster.upper_left_corner.z;

            p_max_max.x = cluster.upper_left_corner.x;
            p_max_max.y = cluster.upper_left_corner.y;
            p_max_max.z = cluster.upper_left_corner.z;

            p_max_min.x = cluster.upper_left_corner.x;
            p_max_min.y = cluster.lower_right_corner.y;
            p_max_min.z = cluster.upper_left_corner.z;

            lines_marker.points.push_back(p_min_min);
            lines_marker.points.push_back(p_min_max);

            lines_marker.points.push_back(p_min_max);
            lines_marker.points.push_back(p_max_max);

            lines_marker.points.push_back(p_max_max);
            lines_marker.points.push_back(p_max_min);

            lines_marker.points.push_back(p_max_min);
            lines_marker.points.push_back(p_min_min);
        }

        // put markers in the marker array
        markers.markers.push_back(points_marker);
        markers.markers.push_back(lines_marker);

        // publish the marker array
        markers_pub_.publish(markers);
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

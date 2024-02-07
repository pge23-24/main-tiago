#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from multi_obstacles_tracker_msgs.msg import ObstacleMeasureStamped, ObstacleMeasureStampedArray 
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

    
class ObstaclesCluster:
    def __init__(self) -> None:       
        # subscribe to the topic /scan_filtered to get raw points cloud
        rospy.Subscriber("/scan_filtered", PointCloud2, self.filteredScanCallback)

        # Publish to the topic /measures
        self.obstacles_array_msg_pub_ = rospy.Publisher("/measures", ObstacleMeasureStampedArray, queue_size=1)

        # Publish to the topic /viz_clusters
        self.viz_clusters_array_msg_pub_ = rospy.Publisher("/viz_clusters", MarkerArray, queue_size=1)

        # save color map to avoid death
        self.color_map = {}
        
        # spin it
        rospy.spin()

    def filteredScanCallback(self, cloud: PointCloud2):
        # measures array msg
        measures_array_msg = ObstacleMeasureStampedArray()

        # get points
        it_points = pc2.read_points(cloud, skip_nans=True)
        points = list(it_points)

        # convert points list to numpy array
        np_points = np.array(points)

        # apply DBSCAN algorithm to get clusters
        clusters = DBSCAN(eps=0.5, min_samples=5).fit(np_points)

        # get np clusters
        list_clusters = []
        np_labels = np.array(clusters.labels_)
        for i in range(1, np.max(np_labels)+1):
            # get the i_th np_cluster
            indices = np.argwhere(np_labels == i)
            list_clusters.append(np_points[indices][:, 0, :])

            # get its mean and covariance
            mean = np.mean(list_clusters[-1], axis=0)
            cov = np.cov(list_clusters[-1].T)

            # create and append measure message
            measure_msg = ObstacleMeasureStamped()

            measure_msg.header = cloud.header

            measure_msg.mean[0] = mean[0]
            measure_msg.mean[1] = mean[1]

            measure_msg.covariance[0] = cov[0, 0]
            measure_msg.covariance[1] = cov[0, 1]
            measure_msg.covariance[2] = cov[1, 0]
            measure_msg.covariance[3] = cov[1, 1]

            measures_array_msg.measures.append(measure_msg)

        # publish measures array
        self.obstacles_array_msg_pub_.publish(measures_array_msg)

        # # plot it
        # markers_array_msg = MarkerArray()
        # for j, np_cluster in enumerate(list_clusters):
        #     if j not in self.color_map:
        #         self.color_map[j] = (np.random.rand(1), np.random.rand(1), np.random.rand(1))
        #     marker = Marker()
        #     marker.id = j
        #     marker.header = cloud.header
        #     marker.lifetime.secs = 0
        #     marker.lifetime.nsecs = 1_000_000
        #     marker.action = Marker.ADD
        #     marker.color.a = 1
        #     marker.color.r = self.color_map[j][0][0]
        #     marker.color.g = self.color_map[j][1][0]
        #     marker.color.b = self.color_map[j][2][0]
        #     marker.type = Marker.POINTS
        #     marker.scale.x = 0.1
        #     marker.scale.y = 0.1
        #     for i in range(np_cluster.shape[0]):
        #         marker.points.append(Point(x=np_cluster[i, 0], y=np_cluster[i, 1], z=0))
        #     markers_array_msg.markers.append(marker)
        # self.viz_clusters_array_msg_pub_.publish(markers_array_msg)
        

if __name__ == "__main__":
    rospy.init_node('obstacles_cluster')

    try:
        ne = ObstaclesCluster()
    except rospy.ROSInterruptException:
        pass
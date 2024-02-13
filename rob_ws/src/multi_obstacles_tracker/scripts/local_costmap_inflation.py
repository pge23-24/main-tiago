#!/usr/bin/env python
import rospy
import threading
import numpy as np
import message_filters
import tf
import copy
from nav_msgs.msg import OccupancyGrid, Odometry
from multi_obstacles_tracker_msgs.msg import CameraDetectionStampedArray
from scipy.ndimage import gaussian_filter

costmap_mutex = threading.Lock()
detections_mutex = threading.Lock()


class LocalCostMapInflation:
    def __init__(self) -> None:
        # subscribe to the topic /move_base/local_costmap/to_inflate_costmap
        self.costmap_sub_ = rospy.Subscriber(
            "/to_inflate_costmap", OccupancyGrid, self.costmapCallback
        )

        # subscribe to the topic /information_1
        self.camera_detection_sub_ = rospy.Subscriber(
            "/camera_detection_1", CameraDetectionStampedArray, self.detectionsCallback
        )

        # subscribe to the topic /mobile_base_controller/odom
        self.odom_sub_ = rospy.Subscriber(
            "/mobile_base_controller/odom",
            Odometry,
            self.inflateCallback,
        )

        # Publish to the topic /measures
        self.costmap_pub_ = rospy.Publisher(
            "/move_base/local_costmap/costmap", OccupancyGrid, queue_size=1
        )

        # the current costmap
        self.costmap = None

        # the current odom
        self.detections = None

        # inflation dist
        self.inflation_dist = 1

        # spin it
        rospy.spin()

    def detectionsCallback(self, detections: CameraDetectionStampedArray):
        with detections_mutex:
            self.detections = detections
        # print('detections acquired')

    def costmapCallback(self, costmap: OccupancyGrid):
        with costmap_mutex:
            self.costmap = costmap
        # print("costmap received")

    def inflateCallback(self, odom: Odometry):
        # copy the current map
        # print("synch")
        with costmap_mutex:
            inflated_costmap = copy.copy(self.costmap)
        if self.costmap is None:
            # print("aborted costmap")
            return
        inflated_costmap_data_np = np.array(list(inflated_costmap.data))

        # init the global costmap mask
        global_costmap_mask = copy.copy(inflated_costmap)
        global_costmap_data_np = np.array(list(inflated_costmap.data)) * 0

        # get metadata of the map
        width_max, height_max = (
            inflated_costmap.info.width,
            inflated_costmap.info.height,
        )
        resolution = inflated_costmap.info.resolution

        # get rotation angle between odom and map
        with detections_mutex:
            detections = self.detections
        if detections is None:
            # print("aborted odom")
            return
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        z_tf = euler_angles[2]

        for detection in detections.detections:
            # check if class is a person
            if detection.classification != "person":
                continue

            # get covariance matrix components
            a = detection.covariance[0]
            b = detection.covariance[1]
            c = detection.covariance[3]

            # a = 1
            # b = 0
            # c = 0

            # get minor major axes and theta
            lambda_1 = np.abs((a + c) / 2 + np.sqrt(((a - c) ** 2) / 4 + b**2))
            lambda_2 = np.abs((a + c) / 2 - np.sqrt(((a - c) ** 2) / 4 + b**2))
            theta = 0
            if b == 0 and a >= c:
                theta = 0
            elif b == 0 and a < c:
                theta = np.pi / 2
            else:
                theta = np.arctan2(lambda_1 - a, b)

            theta = -theta

            # get center of the ellipse in meter
            ellipse_center = detection.coordinates[0] * np.array(
                [
                    np.cos(-detection.coordinates[1] * np.pi / 180.0),
                    np.sin(-detection.coordinates[1] * np.pi / 180.0),
                ]
            )

            # exclude elements out of the local costmap
            if (np.abs(ellipse_center[0]) > (width_max * resolution)) or (
                np.abs(ellipse_center[1]) > (height_max * resolution)
            ):
                # skip the current detection
                continue

            # converts meter to pixels
            ellipse_center_px = (ellipse_center / resolution).astype(int)
            width_px = ((2 * lambda_2) / resolution).astype(int)
            height_px = ((2 * lambda_1) / resolution).astype(int)

            # create an empty mask of the costmap
            costmap_mask = copy.copy(inflated_costmap)
            costmap_mask_data_np = np.array(list(costmap_mask.data)) * 0

            # browse through each pixel of the bounding box in the costmap mask
            for x in range(-width_px // 2, width_px // 2):
                for y in range(-int(height_px // 2), int(height_px // 2)):
                    # compute tf matrix from ellipse frame to robot frame
                    T_r_e = np.array(
                        [
                            [np.cos(theta), -np.sin(theta), ellipse_center_px[0]],
                            [np.sin(theta), np.cos(theta), ellipse_center_px[1]],
                            [0, 0, 1],
                        ]
                    )

                    T_m_r = np.array(
                        [
                            [np.cos(z_tf), -np.sin(z_tf), width_max / 2],
                            [np.sin(z_tf), np.cos(z_tf), height_max / 2],
                            [0, 0, 1],
                        ]
                    )

                    # compute pixel position of the bounding box
                    X_px = T_m_r @ T_r_e @ np.array([x, y, 1]).T
                    x_px = int(X_px[0])
                    y_px = int(X_px[1])

                    # check if the pixel is in the costmap
                    if x_px >= width_max or y_px >= height_max:
                        # skip this pixel
                        continue

                    if x_px < 0 or y_px < 0:
                        # skip this pixel
                        continue

                    # write the binary pixel to the mask
                    costmap_mask_data_np[x_px + y_px * height_max] = 1

            # update data of the global costmap mask
            global_costmap_data_np |= costmap_mask_data_np

        # scale global costmap data
        global_costmap_data_np *= 100

        # reshape global costmap data
        reshaped_global_costmap_data_np = np.reshape(
            global_costmap_data_np, (width_max, height_max)
        )

        # apply gaussian filter to the global costmap
        inflation_pixel = int(self.inflation_dist / resolution)
        filtered_reshaped_global_costmap_data_np = gaussian_filter(
            reshaped_global_costmap_data_np, sigma=2, radius=inflation_pixel
        )

        # unshape the filtered global costmap
        filtered_global_costmap_data_np = np.reshape(
            filtered_reshaped_global_costmap_data_np, (width_max * height_max,)
        )

        # get the max between ros local costmap and human detections costmap
        global_costmap_data_np = np.maximum(
            filtered_global_costmap_data_np, inflated_costmap_data_np
        )

        # fill inflated costmap data
        inflated_costmap.data = tuple((global_costmap_data_np).tolist())

        # publish it!
        self.costmap_pub_.publish(inflated_costmap)


if __name__ == "__main__":
    rospy.init_node("local_costmap_inflation")

    try:
        lcmi = LocalCostMapInflation()
    except rospy.ROSInterruptException:
        pass

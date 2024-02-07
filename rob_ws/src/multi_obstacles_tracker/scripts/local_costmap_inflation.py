#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from nav_msgs.msg import OccupancyGrid
from multi_obstacles_tracker_msgs.msg import CameraDetectionStampedArray

    
class LocalCostMapInflation:
    def __init__(self) -> None:       
        # subscribe to the topic /move_base/local_costmap/to_inflate_costmap to get raw points cloud
        self.costmap_sub_ = message_filters.Subscriber("/to_inflate_costmap", OccupancyGrid)

        # subscribe to the topic /information_1 to get raw points cloud
        self.camera_detection_sub_ = message_filters.Subscriber("/information_1", CameraDetectionStampedArray)

        # Publish to the topic /measures
        self.costmap_pub_ = rospy.Publisher("/move_base/local_costmap/costmap", OccupancyGrid, queue_size=1)

        # create synchroniser
        ts = message_filters.TimeSynchronizer([self.costmap_sub_, self.camera_detection_sub_], 10)
        ts.registerCallback(self.inflateCallback) 
        
        # spin it
        rospy.spin()

    def inflateCallback(self, costmap: OccupancyGrid, detections: CameraDetectionStampedArray):
        # copy the current map
        inflated_costmap = costmap
        inflated_costmap_data_np = np.array(list(inflated_costmap.data))

        # init the global costmap mask
        global_costmap_mask = inflated_costmap
        global_costmap_data_np = np.array(list(inflated_costmap.data)) * 0

        # get metadata of the map
        width, height = inflated_costmap.info.width, inflated_costmap.info.height
        resolution = inflated_costmap.info.resolution

        for detection in detections.detections:
            # get covariance matrix components
            a = detection.covariance[0]
            b = detection.covariance[1]
            c = detection.covariance[2]
            
            # get minor major axes and theta
            lambda_1 = 2*np.sqrt(9.21*np.abs((a+c)/2 + np.sqrt(((a-c)**2)/4 + b**2)))
            lambda_2 = 2*np.sqrt(9.21*np.abs((a+c)/2 - np.sqrt(((a-c)**2)/4 + b**2)))
            theta = 0
            if b == 0 and a >= c:
                theta = 0
            elif b == 0 and a < c:
                theta = np.pi/2
            else:
                theta = np.arctan2(lambda_1-a, b)

            print(theta - detection.coordinates[1])

            # get center of the ellipse in meter
            ellipse_center = detection.coordinates[0] * np.array([np.cos(detection.coordinates[1]), np.sin(detection.coordinates[1])])

            # exclude elements out of the local costmap
            if (np.abs(ellipse_center[0]) > width * resolution) or (np.abs(ellipse_center[1]) > height * resolution):
                # skip the current detection
                continue

            # get pos of bounding box top left corner of the ellipse in meter
            OA = 1.96 * np.sqrt(lambda_1) * np.array([np.cos(theta), np.sin(theta)])
            OB = 1.96 * np.sqrt(lambda_2) * np.array([np.sin(theta), -np.cos(theta)])
            top_left_corner = OA - OB + ellipse_center

            # converts meter to pixels
            ellipse_center_px = ellipse_center / resolution
            top_left_corner_px = top_left_corner / resolution
            width_px = (2 * lambda_2) / resolution
            height_px = (2 * lambda_1) / resolution

            # create an empty mask of the costmap
            costmap_mask = inflated_costmap
            costmap_mask_data_np = np.array(list(inflated_costmap.data)) * 0
        
            # browse through each pixel of the bounding box in the costmap mask
            for x in range(-width_px//2, width_px//2):
                for y in range(-height_px//2, height_px//2):
                    # compute pixel position of the bounding box
                    x_px = int(x * np.cos(theta) + width_px//2 + top_left_corner[0]) 
                    y_px = int(y * np.sin(theta) + height_px//2 + top_left_corner[1])

                    # check if the pixel is in the costmap
                    if x_px > width or y_px > height:
                        # skip this pixel
                        continue
                    
                    # write the binary pixel to the mask
                    costmap_mask_data_np[x_px + y_px * height] = 1
            
            # update data of the global costmap mask
            global_costmap_data_np |= costmap_mask_data_np

            print(global_costmap_data_np)
        
        # # reshape global costmap data
        # reshaped_global_costmap_data_np = np.reshape(global_costmap_data_np, (width, height)) 

        # # scale global costmap data
        # reshaped_global_costmap_data_np *= 255

        # apply global costmap mask to the inflated costmap
        inflated_costmap = np.max(inflated_costmap_data_np, global_costmap_data_np)

        self.costmap_pub_.publish(inflated_costmap)

        

if __name__ == "__main__":
    rospy.init_node('local_costmap_inflation')

    try:
        lcmi = LocalCostMapInflation()
    except rospy.ROSInterruptException:
        pass
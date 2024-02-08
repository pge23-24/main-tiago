#!/usr/bin/env python
import rospy
import message_filters
from nav_msgs.msg import OccupancyGrid

    
class LocalCostMapInflation:
    def __init__(self) -> None:       
        # subscribe to the topic /move_base/local_costmap/to_inflate_costmap to get raw points cloud
        message_filters.Subscriber("/to_inflate_costmap", OccupancyGrid, self.costmapCallback)

        # subscribe to the topic /move_base/local_costmap/to_inflate_costmap to get raw points cloud
        message_filters.Subscriber("/topic_vision", OccupancyGrid, self.costmapCallback)

        # Publish to the topic /measures
        self.costmap_pub_ = rospy.Publisher("/move_base/local_costmap/costmap", OccupancyGrid, queue_size=1)
        
        # spin it
        rospy.spin()

    def costmapCallback(self, costmap: OccupancyGrid):
        inflated_costmap = costmap
        
        # width, height = inflated_costmap.info.width, inflated_costmap.info.height
        # data = list(inflated_costmap.data)

        # for x in range(width):
        #     for y in range(height):
        #         data[x + y * width] = 0

        # inflated_costmap.data = tuple(data)

        self.costmap_pub_.publish(inflated_costmap)

        

if __name__ == "__main__":
    rospy.init_node('local_costmap_inflation')

    try:
        lcmi = LocalCostMapInflation()
    except rospy.ROSInterruptException:
        pass
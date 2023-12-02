# This message holds informations of a cluster of points.

# Time of sensor data acqusition, coordinate frame ID.
Header header

# Coordinate of the obstacle's barycenter in the frame given in the header.
geometry_msgs/Point32 barycenter

# Width and height of the obstacles.
float32 width
float32 height
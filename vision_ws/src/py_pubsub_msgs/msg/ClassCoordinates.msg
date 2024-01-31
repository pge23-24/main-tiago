# This message is used to send the angle of a detected class
# Timestamp
std_msgs/Header header
# Class name
string classification
# Angle of the detected class
float32 angle
# Covariance matrix of the ellipse
float32[4] covariance_matrix
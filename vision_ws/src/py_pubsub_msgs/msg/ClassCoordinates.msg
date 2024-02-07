#/ This message holds informations of a camera's detection.
#/ Time of sensor data acqusition, coordinate frame ID.
std_msgs/Header header

#/ Class of the detection
string classification

#/ Polar coordinates (rho, theta) of the detection in the frame given in the header.
float32[2] coordinates

#/ Statistical covariance of the detection in the frame given in the header.
float32[4] covariance
# This message holds informations of a obstacle's measure.

# Time of sensor data acqusition, coordinate frame ID.
Header header

# Statistical average of the obstacle in the frame given in the header.
float32[2] mean

# Statistical covariance of the obstacle in the frame given in the header.
float32[4] covariance
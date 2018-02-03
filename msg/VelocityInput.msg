Header header
uint32 id                     ## Sensor id
geometry_msgs/Vector3 vel     ## Measurement
float64[9] cov                  ## Measurement Covariance
geometry_msgs/Quaternion q    ## Rotation from IMU frame to sensor frame

Header header

uint32 id
geometry_msgs/Vector3 vel # Velocity measurement from sensor (x,y,z)
geometry_msgs/Vector3 cov # Covariance Matrix Diagonal (E[xx], E[yy], E[zz])
geometry_msgs/Quaternion q # Quaternion rotation from IMU frame to sensor frame
geometry_msgs/Vector3 p # Sensor origin position in IMU frame
bool[3] valid # Valid velocity vector terms (x, y, z)

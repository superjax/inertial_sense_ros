# GPS status flags
uint32 GPS_STATUS_FIX_TYPE_NO_FIX               = 0
uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = 256
uint32 GPS_STATUS_FIX_TYPE_2D_FIX               = 512
uint32 GPS_STATUS_FIX_TYPE_3D_FIX               = 768
uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = 1024
uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = 1280
uint32 GPS_STATUS_FIX_TYPE_RESERVED1            = 1536
uint32 GPS_STATUS_FIX_TYPE_RESERVED2            = 1792

uint32 GPS_STATUS_FIX_STATUS_FIX_OK             = 65536

Header header
int8 num_sat
uint32 fix_type
int32 cno
float64 latitude
float64 longitude
float64 altitude
float32 hMSL
float32 hAcc
float32 vAcc
float32 pDop
geometry_msgs/Vector3 linear_velocity
float32 ground_speed_2d
float32 ground_speed_3d
float32 sAcc
float32 course
float32 cAcc
uint32 messages_per_second


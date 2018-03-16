#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>

#include "InertialSense.h"
#include "ISDisplay.h"
#include "ISUtilities.h"

#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "inertial_sense/GPS.h"
#include "inertial_sense/GPSInfo.h"
#include "inertial_sense/DThetaVel.h"
#include "nav_msgs/Odometry.h"

# define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970
# define LEAP_SECONDS 18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

class InertialSenseROS
{
public:
  InertialSenseROS();
  void callback(p_data_t* data);
  void update();

private:

  // Serial Port Configuration
  std::string port_;
  int baudrate_;
  ros::Duration INS_local_offset_;
  double GPS_towOffset_; // The offset between GPS time-of-week and local time on the uINS
  uint64_t GPS_week_;

  nvm_flash_cfg_t flash_cfg_;

  std::string frame_id_;

  // ROS Stream handling
  typedef struct
  {
    bool stream_on;
    int stream_rate;
    ros::Publisher pub;
    ros::Publisher pub2;
  } ros_stream_t;

  ros_stream_t INS_;
  void INS1_callback();
  void INS2_callback();

  ros_stream_t IMU_;
  void IMU_callback();

  ros_stream_t GPS_;
  void GPS_callback();

  ros_stream_t GPS_info_;
  void GPS_Info_callback();

  ros_stream_t mag_;
  void mag_callback(int mag_number);

  ros_stream_t baro_;
  void baro_callback();

  ros_stream_t dt_vel_;
  void dtheta_vel_callback();


  // Data to hold on to in between callbacks
  sensor_msgs::Imu imu1_msg, imu2_msg;
  nav_msgs::Odometry odom_msg;
  inertial_sense::GPS gps_msg;
  inertial_sense::GPSInfo gps_info_msg;


  // Data Struct received from uINS
  uDatasets d_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  InertialSense inertialSenseInterface_;
};

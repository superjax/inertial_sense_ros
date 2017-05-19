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
#include "sensor_msgs/NavSatFix.h"
#include "inertial_sense/GPS.h"
#include "inertial_sense/GPSInfo.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"

# define GPS_UTC_OFFSET 315964782 // as of 2017


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
  ros::Duration IMU_offset_;
  bool first_IMU_message_ = true;
  bool got_GPS_fix_ = false;
  double GPS_to_week_offset_;

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

  // Data to hold on to in between callbacks
  sensor_msgs::Imu imu1_msg, imu2_msg;
  nav_msgs::Odometry odom_msg;
  inertial_sense::GPS gps_msg;
  inertial_sense::GPSInfo gps_info_msg;
  uint64_t GPS_week_seconds;


  // Data Struct received from uINS
  uDatasets d_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  InertialSense inertialSenseInterface_;
};

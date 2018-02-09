#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>

#include "ISComm.h"
#include "serialPortPlatform.h"

#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "inertial_sense/GPS.h"
#include "inertial_sense/GPSInfo.h"
#include "inertial_sense/DThetaVel.h"
#include "inertial_sense/VelocityInput.h"
#include "nav_msgs/Odometry.h"

# define GPS_UTC_OFFSET 315964782 // as of 2017

#define BUFFER_SIZE 2048


class InertialSenseROS //: SerialListener
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
  is_comm_instance_t comm;
  nvm_flash_cfg_t flash_cfg_;

  std::string frame_id_;

  // Inputs
  ros::Subscriber vel_sub_;
  void velocity_callback(const inertial_sense::VelocityInputConstPtr &msg);

  // ROS Stream handling
  typedef struct
  {
    bool stream_on;
    int stream_rate;
    ros::Publisher pub;
    ros::Publisher pub2;
  } ros_stream_t;

  void request_data(uint32_t did, float update_rate);

  ros_stream_t INS_;
  void INS1_callback(const ins_1_t* const msg);
  void INS2_callback(const ins_2_t* const msg);

  ros_stream_t IMU_;
  void IMU_callback(const dual_imu_t* const msg);

  ros_stream_t GPS_;
  void GPS_callback(const gps_nav_t* const msg);

  ros_stream_t GPS_info_;
  void GPS_Info_callback(const gps_sat_t* const msg);

  ros_stream_t mag_;
  void mag_callback(const magnetometer_t* const msg, int mag_number);

  ros_stream_t baro_;
  void baro_callback(const barometer_t* const msg);

  ros_stream_t dt_vel_;
  void dtheta_vel_callback(const dual_imu_dtheta_dvel_t* const msg);


  // Data to hold on to in between callbacks
  sensor_msgs::Imu imu1_msg, imu2_msg;
  nav_msgs::Odometry odom_msg;
  inertial_sense::GPS gps_msg;
  inertial_sense::GPSInfo gps_info_msg;
  uint64_t GPS_week_seconds;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Serial Connection to uINS
//  Serial* serial_;
  uint8_t message_buffer_[BUFFER_SIZE];
  is_comm_instance_t s_comm_;
  serial_port_t serial_;

//  InertialSense inertialSenseInterface_;
};

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>

#include "ISComm.h"
//#include "serial.h"
#include "serialPortPlatform.h"

#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "inertial_sense/GPS.h"
#include "inertial_sense/GPSInfo.h"
#include "inertial_sense/PreIntIMU.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"

# define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
# define LEAP_SECONDS 18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast) 
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS) 

#define BUFFER_SIZE 2048


class InertialSenseROS //: SerialListener
{
  typedef enum
  {
    NMEA_GPGGA = 0x01,
    NMEA_GPGLL = 0x02,
    NMEA_GPGSA = 0x04,
    NMEA_GPRMC = 0x08,
    NMEA_SER0 = 0x01,
    NMEA_SER1 = 0x02
  } NMEA_message_config_t;
      
public:
  InertialSenseROS();
  void callback(p_data_t* data);
  void update();

private:
  
  void initialize_uINS();
  template<typename T> void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
  template <typename T>  void set_flash_config(std::string param_name, uint32_t offset, T def);
  void get_flash_config();
  void reset_device();
  void flash_config_callback(const nvm_flash_cfg_t* const msg);
  // Serial Port Configuration
  std::string port_;
  int baudrate_;
  
  // Time sync variables
  double INS_local_offset_ = 0.0;
  bool got_first_message_ = false;
  double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS 
  uint64_t GPS_week_ = 0;

  std::string frame_id_;

  // ROS Stream handling
  typedef struct
  {
    bool enabled;
    ros::Publisher pub;
    ros::Publisher pub2;
  } ros_stream_t;

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
  void preint_IMU_callback(const preintegrated_imu_t * const msg);
  
  ros::ServiceServer mag_cal_srv_;
  ros::ServiceServer multi_mag_cal_srv_;
  bool perform_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);


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
  is_comm_instance_t comm_;
  uint8_t message_buffer_[BUFFER_SIZE];
  serial_port_t serial_;
  bool got_flash_config = false;
  nvm_flash_cfg_t flash_; // local copy of flash config

//  InertialSense inertialSenseInterface_;
};


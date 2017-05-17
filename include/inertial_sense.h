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
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"



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

  // Data to hold on to in between callbacks
  sensor_msgs::Imu imu1_msg, imu2_msg;
  nav_msgs::Odometry odom_msg;
  inertial_sense::GPS gps;

//  ros_stream_t mag1_;
//  void mag1_timer_callback(const ros::TimerEvent&);

//  ros_stream_t mag2_;
//  void mag2_timer_callback(const ros::TimerEvent&);

//  ros_stream_t baro_;
//  void baro_timer_callback(const ros::TimerEvent&);

//  ros_stream_t sensors_;
//  void sensors_timer_callback(const ros::TimerEvent&);

//  ros_stream_t coning_and_sculling_;
//  void CnS_timer_callback(const ros::TimerEvent&);


  // Data Struct received from uINS
  uDatasets d_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;



  InertialSense inertialSenseInterface_;
};

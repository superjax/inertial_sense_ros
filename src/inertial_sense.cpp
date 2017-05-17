#include "inertial_sense.h"
#include <chrono>

#include <ros/console.h>

static void data_callback(InertialSense* i, p_data_t* data);

InertialSenseROS::InertialSenseROS() :
  nh_(), nh_private_("~")
{
  nh_private_.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate_, 3000000);
  nh_private_.param<std::string>("frame_id", frame_id_, "body");


  // Connect to the uINS
  if (!inertialSenseInterface_.Open(port_.c_str(), baudrate_))
  {
    ROS_ERROR("Unable to open uINS-2 on port %s at %d baud", port_.c_str(), baudrate_);
    return;
  }
  else
  {
    ROS_INFO_STREAM("InertialSense uINS-2 Connected on port " << port_.c_str() << " at " << baudrate_ << " baud");
  }
  inertialSenseInterface_.StopBroadcasts();	// Stop streaming any prior messages


  // Set up the INS ROS stream
  nh_private_.param<bool>("sINS", INS_.stream_on, true);
  nh_private_.param<int>("sINS_rate", INS_.stream_rate, 100);
  if (INS_.stream_on)
  {
    INS_.pub = nh_.advertise<nav_msgs::Odometry>("ins", 1);
    inertialSenseInterface_.BroadcastBinaryData(DID_INS_1, (int)(1000/INS_.stream_rate), &data_callback);
    inertialSenseInterface_.BroadcastBinaryData(DID_INS_2, (int)(1000/INS_.stream_rate), &data_callback);
    inertialSenseInterface_.BroadcastBinaryData(DID_DUAL_IMU, (int)(1000/INS_.stream_rate), &data_callback);
  }



  // Set up the IMU ROS stream
  nh_private_.param<bool>("sIMU", IMU_.stream_on, true);
  nh_private_.param<int>("sIMU1_rate", IMU_.stream_rate, 100);
  if (IMU_.stream_on)
  {
    IMU_.pub = nh_.advertise<sensor_msgs::Imu>("imu1", 1);
    IMU_.pub2 = nh_.advertise<sensor_msgs::Imu>("imu2", 1);
    uint32_t update_rate = 1000/IMU_.stream_rate;
    if (INS_.stream_on)
    {
      update_rate = (INS_.stream_rate > IMU_.stream_rate) ? 1000/IMU_.stream_rate : update_rate;
    }
    inertialSenseInterface_.BroadcastBinaryData(DID_DUAL_IMU, update_rate, &data_callback);
  }

  // Set up the GPS ROS stream
  nh_private_.param<bool>("sGPS", GPS_.stream_on, true);
  nh_private_.param<int>("sGPS_rate", GPS_.stream_rate, 10);
  if (GPS_.stream_on)
  {
    GPS_.pub = nh_.advertise<inertial_sense::GPS>("gps", 1);
    inertialSenseInterface_.BroadcastBinaryData(DID_GPS, 1000/GPS_.stream_rate, &data_callback);

  }

  //  nh_private_.param<bool>("smag1", stream_mag1_, false);
  //  nh_private_.param<int>("smag1_rate", stream_mag1_rate_, 0);

  //  nh_private_.param<bool>("smag2", stream_mag2_, false);
  //  nh_private_.param<int>("smag2_rate", stream_mag2_rate_, 0);

  //  nh_private_.param<bool>("sbaro", stream_baro_, true);
  //  nh_private_.param<int>("sbaro_rate", stream_baro_rate_, 100);

  //  nh_private_.param<bool>("ssensors", stream_sensors_, false);
  //  nh_private_.param<int>("ssensors_rate", stream_sensors_rate_, 0);

  //  nh_private_.param<bool>("sdelta_theta_vel", stream_delta_theta_vel_, true);
  //  nh_private_.param<int>("sdelta_theta_vel_rate", stream_delta_theta_vel_rate_, 500);

  // ask for device info every 2 seconds
  inertialSenseInterface_.BroadcastBinaryData(DID_DEV_INFO, 2000, &data_callback);
}

void InertialSenseROS::INS1_callback()
{
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.position.x = d_.ins1.ned[0];
  odom_msg.pose.pose.position.y = d_.ins1.ned[1];
  odom_msg.pose.pose.position.z = d_.ins1.ned[2];
}


void InertialSenseROS::INS2_callback()
{
  uint64_t seconds = d_.ins2.week*7*24*3600 + floor(d_.ins2.timeOfWeek*7.0*24.0*3600.0);
  uint64_t nsec = (d_.ins2.timeOfWeek*7.0*24.0*3600.0 - floor(d_.ins2.timeOfWeek*7.0*24.0*3600.0))*1e9;
  //  odom_msg.header.stamp = ros::Time(seconds, nsec);
  odom_msg.header.stamp = ros::Time::now();

  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.orientation.w = d_.ins2.qn2b[0];
  odom_msg.pose.pose.orientation.x = d_.ins2.qn2b[1];
  odom_msg.pose.pose.orientation.y = d_.ins2.qn2b[2];
  odom_msg.pose.pose.orientation.z = d_.ins2.qn2b[3];

  odom_msg.twist.twist.linear.x = d_.ins2.uvw[0];
  odom_msg.twist.twist.linear.y = d_.ins2.uvw[1];
  odom_msg.twist.twist.linear.z = d_.ins2.uvw[2];

  odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
  odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
  odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;
  INS_.pub.publish(odom_msg);
}
void InertialSenseROS::IMU_callback()
{
  // This needs to be adjusted to ros::Time
  //  imu1_msg.header.stamp = imu2_msg.header.stamp = ros::Time(d_.dualImu.time);
  imu1_msg.header.stamp = imu2_msg.header.stamp = ros::Time::now();
  imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

  imu1_msg.angular_velocity.x = d_.dualImu.I[0].pqr[0];
  imu1_msg.angular_velocity.y = d_.dualImu.I[0].pqr[1];
  imu1_msg.angular_velocity.z = d_.dualImu.I[0].pqr[2];
  imu1_msg.linear_acceleration.x = d_.dualImu.I[0].acc[0];
  imu1_msg.linear_acceleration.y = d_.dualImu.I[0].acc[1];
  imu1_msg.linear_acceleration.z = d_.dualImu.I[0].acc[2];

  imu2_msg.angular_velocity.x = d_.dualImu.I[1].pqr[0];
  imu2_msg.angular_velocity.y = d_.dualImu.I[1].pqr[1];
  imu2_msg.angular_velocity.z = d_.dualImu.I[1].pqr[2];
  imu2_msg.linear_acceleration.x = d_.dualImu.I[1].acc[0];
  imu2_msg.linear_acceleration.y = d_.dualImu.I[1].acc[1];
  imu2_msg.linear_acceleration.z = d_.dualImu.I[1].acc[2];

  if (IMU_.stream_on)
  {
    IMU_.pub.publish(imu1_msg);
    IMU_.pub2.publish(imu2_msg);
  }
}


void InertialSenseROS::GPS_callback()
{
  gps.fix = d_.gps.pos.status & GPS_STATUS_FIX_STATUS_FIX_OK;
  gps.header.stamp = ros::Time::now();
  gps.header.frame_id =frame_id_;
  uint8_t* num_sat = (uint8_t*)&d_.gps.pos.status;
  gps.num_sat = *num_sat;
  gps.cno = d_.gps.pos.cno;
  gps.latitude = d_.gps.pos.lla[0];
  gps.longitude = d_.gps.pos.lla[1];
  gps.altitude = d_.gps.pos.lla[2];
  gps.hMSL = d_.gps.pos.hMSL;
  gps.hAcc = d_.gps.pos.hAcc;
  gps.vAcc = d_.gps.pos.vAcc;
  gps.pDop = d_.gps.pos.pDop;
  gps.linear_velocity.x = d_.gps.vel.ned[0];
  gps.linear_velocity.y = d_.gps.vel.ned[1];
  gps.linear_velocity.z = d_.gps.vel.ned[2];
  gps.ground_speed_2d = d_.gps.vel.s2D;
  gps.ground_speed_3d = d_.gps.vel.s3D;
  gps.course = d_.gps.vel.course;
  gps.cAcc = d_.gps.vel.cAcc;
  GPS_.pub.publish(gps);
}

void InertialSenseROS::callback(p_data_t* data)
{
  copyDataPToStructP(&d_, data, sizeof(uDatasets));
  switch(data->hdr.id)
  {
  case DID_INS_1:
    INS1_callback();
    break;
  case DID_INS_2:
    INS2_callback();
    break;

  case DID_DUAL_IMU:
    IMU_callback();
    break;

  case DID_GPS:
    GPS_callback();
    break;
  }

}

void InertialSenseROS::update()
{
  inertialSenseInterface_.Update();
}




// I really, really hate this global pointer
InertialSenseROS* ISROSPTr;
static void data_callback(InertialSense* i, p_data_t* data)
{
  ISROSPTr->callback(data);
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "inertial_sense_node");
  InertialSenseROS thing;
  ISROSPTr = &thing;

  while(ros::ok())
  {
    thing.update();
    ros::spinOnce();
  }

  return 0;
}

#include "inertial_sense.h"
#include <chrono>
#include <stddef.h>

#include <ros/console.h>

//static void data_callback(InertialSense* i, p_data_t* data, int pHandle);

InertialSenseROS::InertialSenseROS() :
    nh_(), nh_private_("~"), IMU_offset_(0,0), GPS_to_week_offset_(0)
{
    nh_private_.param<std::string>("port", port_, "/dev/ttyUSB0");
    nh_private_.param<int>("baudrate", baudrate_, 3000000);
    nh_private_.param<std::string>("frame_id", frame_id_, "body");

    /// Connect to the uINS

    ROS_INFO("Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    serial_ = new Serial(port_, baudrate_);

    try
    {
      serial_->open();
      serial_->register_listener(this);
    }
    catch (SerialException e)
    {
      ROS_FATAL("%s", e.what());
      ros::shutdown();
    }
    ROS_INFO("Connected to uINS on \"%s\", at %d baud", port_.c_str(), baudrate_);

    is_comm_init(message_buffer_, sizeof(message_buffer_));

    // Stop all broadcasts
    uint32_t messageSize = is_comm_stop_broadcasts();
    serial_->write(message_buffer_, messageSize);

    /// Configure the uINS

    // Change the flash based on parameters
    std::vector<double> INS_rpy(3, 0.0);
    std::vector<double> INS_xyz(3, 0.0);
    std::vector<double> GPS_ant_xyz(3, 0.0);
    std::vector<double> GPS_ref_lla(3, 0.0);

    if (nh_private_.hasParam("INS_rpy"))
        nh_private_.getParam("INS_rpy", INS_rpy);
    if (nh_private_.hasParam("INS_xyz"))
        nh_private_.getParam("INS_xyz", INS_xyz);
    if (nh_private_.hasParam("GPS_ant_xyz"))
        nh_private_.getParam("GPS_ant_xyz", GPS_ant_xyz);
    if (nh_private_.hasParam("GPS_ref_lla"))
        nh_private_.getParam("GPS_ref_lla", GPS_ref_lla);

    float mag_inclination, mag_declination ,mag_magnitude;
    nh_private_.param<float>("inclination", mag_inclination, 1.14878541071);
    nh_private_.param<float>("declination", mag_declination, 0.20007290992);
    nh_private_.param<float>("mag_magnitude", mag_magnitude, 1.0); // 51.6619nT <-- Not sure how this works

    int dynamic_model;
    nh_private_.param<int>("dynamic_model", dynamic_model, 8);

    float insRotation[3];
    insRotation[0] = INS_rpy[0];
    insRotation[1] = INS_rpy[1];
    insRotation[2] = INS_rpy[2];
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, insRotation), sizeof(float[3]), insRotation);
    serial_->write(message_buffer_, messageSize);

    float insOffset[3];
    insOffset[0] = INS_xyz[0];
    insOffset[1] = INS_xyz[1];
    insOffset[2] = INS_xyz[2];
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, insOffset), sizeof(float[3]), insOffset);
    serial_->write(message_buffer_, messageSize);

    float gps1AntOffset[3];
    gps1AntOffset[0] = GPS_ant_xyz[0];
    gps1AntOffset[1] = GPS_ant_xyz[1];
    gps1AntOffset[2] = GPS_ant_xyz[2];
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, gps1AntOffset), sizeof(float[3]), gps1AntOffset);
    serial_->write(message_buffer_, messageSize);

    float refLla[3];
    refLla[0] = GPS_ref_lla[0];
    refLla[1] = GPS_ref_lla[1];
    refLla[2] = GPS_ref_lla[2];
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, refLla), sizeof(float[3]), refLla);
    serial_->write(message_buffer_, messageSize);

    float magInclination = mag_inclination;
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, magInclination), sizeof(float), &magInclination);
    serial_->write(message_buffer_, messageSize);

    float magDeclination = mag_declination;
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, magDeclination), sizeof(float), &magDeclination);
    serial_->write(message_buffer_, messageSize);

    uint32_t insDynModel = dynamic_model;
    messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, insDynModel), sizeof(uint32_t), &insDynModel);
    serial_->write(message_buffer_, messageSize);

    // Set up the INS streams
    nh_private_.param<bool>("sINS", INS_.stream_on, true);
    nh_private_.param<int>("sINS_rate", INS_.stream_rate, 1000);
    if (INS_.stream_on)
    {
        INS_.pub = nh_.advertise<nav_msgs::Odometry>("ins", 1);
        request_data(DID_INS_1, INS_.stream_rate);
        request_data(DID_INS_2, INS_.stream_rate);
        request_data(DID_DUAL_IMU, INS_.stream_rate);
    }

    // Set up the IMU ROS stream
    nh_private_.param<bool>("sIMU", IMU_.stream_on, false);
    nh_private_.param<int>("sIMU1_rate", IMU_.stream_rate, 100);
    if (IMU_.stream_on)
    {
        IMU_.pub = nh_.advertise<sensor_msgs::Imu>("imu1", 1);
        IMU_.pub2 = nh_.advertise<sensor_msgs::Imu>("imu2", 1);
        uint32_t update_rate = IMU_.stream_rate;
        if (INS_.stream_on)
        {
            update_rate = (INS_.stream_rate > IMU_.stream_rate) ? 1000/IMU_.stream_rate : update_rate;
        }
        request_data(DID_DUAL_IMU, update_rate);
    }

    // Set up the GPS ROS stream
    nh_private_.param<bool>("sGPS", GPS_.stream_on, false);
    nh_private_.param<int>("sGPS_rate", GPS_.stream_rate, 10);
    if (GPS_.stream_on)
    {
        GPS_.pub = nh_.advertise<inertial_sense::GPS>("gps", 1);
        request_data(DID_GPS_NAV, GPS_.stream_rate);
    }

     // Set up the GPS info ROS stream
     nh_private_.param<bool>("sGPS_info", GPS_info_.stream_on, false);
     nh_private_.param<int>("sGPS_info_rate", GPS_info_.stream_rate, 10);
     if (GPS_info_.stream_on)
     {
         GPS_info_.pub = nh_.advertise<inertial_sense::GPSInfo>("gps/info", 1);
         request_data(DID_GPS1_SAT, GPS_info_.stream_rate);
     }

    // Set up the magnetometer ROS stream
    nh_private_.param<bool>("smag", mag_.stream_on, false);
    nh_private_.param<int>("smag_rate", mag_.stream_rate, 100);
    if (mag_.stream_on)
    {
        mag_.pub = nh_.advertise<sensor_msgs::MagneticField>("mag1", 1);
        mag_.pub2 = nh_.advertise<sensor_msgs::MagneticField>("mag2", 1);
        request_data(DID_MAGNETOMETER_1, mag_.stream_rate);
        request_data(DID_MAGNETOMETER_2, mag_.stream_rate);
    }

    // Set up the barometer ROS stream
    nh_private_.param<bool>("sbaro", baro_.stream_on, false);
    nh_private_.param<int>("sbaro_rate", baro_.stream_rate, 100);
    if (baro_.stream_on)
    {
        baro_.pub = nh_.advertise<sensor_msgs::FluidPressure>("baro", 1);
        request_data(DID_BAROMETER, baro_.stream_rate);
    }

    // Set up the delta_theta_vel (coning and sculling integral) ROS stream
    nh_private_.param<bool>("sdelta_theta_vel", dt_vel_.stream_on, false);
    nh_private_.param<int>("sdelta_theta_vel_rate", dt_vel_.stream_rate, 100);
    if (dt_vel_.stream_on)
    {
        dt_vel_.pub = nh_.advertise<inertial_sense::DThetaVel>("delta_theta_vel", 1);
        request_data(DID_DUAL_IMU_DTHETA_DVEL, dt_vel_.stream_rate);
    }

    // ask for device info every 2 seconds
    request_data(DID_DEV_INFO, 0.5);
}

void InertialSenseROS::request_data(uint32_t did, float update_rate)
{
  if (update_rate > 1000)
  {
    ROS_ERROR("inertialsense: unable to support stream rates higher than 1kHz");
    update_rate = 1000;
  }

  else
  {
    int messageSize = is_comm_get_data(did, 0, 0, 1000/update_rate);
    serial_->write(message_buffer_, messageSize);
  }
}

void InertialSenseROS::handle_bytes(const uint8_t *bytes, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    uint32_t message_type = is_comm_parse(bytes[i]);
    switch (message_type)
    {
    case DID_NULL:
      // no valid message yet
      break;
    case DID_INS_1:
        INS1_callback((ins_1_t*) message_buffer_);
        break;
    case DID_INS_2:
        INS2_callback((ins_2_t*) message_buffer_);
        break;

    case DID_DUAL_IMU:
        IMU_callback((dual_imu_t*) message_buffer_);
        break;

    case DID_GPS_NAV:
        GPS_callback((gps_nav_t*) message_buffer_);
        break;

    case DID_GPS1_SAT:
        GPS_Info_callback((gps_sat_t*) message_buffer_);
        break;

    case DID_MAGNETOMETER_1:
        mag_callback((magnetometer_t*) message_buffer_, 1);
        break;
    case DID_MAGNETOMETER_2:
        mag_callback((magnetometer_t*) message_buffer_, 2);
        break;

    case DID_BAROMETER:
        baro_callback((barometer_t*) message_buffer_);
        break;

    case DID_DUAL_IMU_DTHETA_DVEL:
        dtheta_vel_callback((dual_imu_dtheta_dvel_t*) message_buffer_);
        break;
    }
  }
}

void InertialSenseROS::INS1_callback(const ins_1_t * const msg)
{
    odom_msg.header.frame_id = frame_id_;

    odom_msg.pose.pose.position.x = msg->ned[0];
    odom_msg.pose.pose.position.y = msg->ned[1];
    odom_msg.pose.pose.position.z = msg->ned[2];
}


void InertialSenseROS::INS2_callback(const ins_2_t * const msg)
{
    uint64_t seconds = GPS_UTC_OFFSET + msg->week*7*24*3600 + floor(msg->timeOfWeek);
    uint64_t nsec = (msg->timeOfWeek - floor(msg->timeOfWeek))*1e9;
    odom_msg.header.stamp = ros::Time(seconds, nsec);

    odom_msg.header.frame_id = frame_id_;

    odom_msg.pose.pose.orientation.w = msg->qn2b[0];
    odom_msg.pose.pose.orientation.x = msg->qn2b[1];
    odom_msg.pose.pose.orientation.y = msg->qn2b[2];
    odom_msg.pose.pose.orientation.z = msg->qn2b[3];

    odom_msg.twist.twist.linear.x = msg->uvw[0];
    odom_msg.twist.twist.linear.y = msg->uvw[1];
    odom_msg.twist.twist.linear.z = msg->uvw[2];

    odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
    odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
    odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;
    INS_.pub.publish(odom_msg);
}
void InertialSenseROS::IMU_callback(const dual_imu_t* const msg)
{

    ros::Time imu_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp IMU messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(msg->time + GPS_to_week_offset_);
        uint64_t nsec = (msg->time + GPS_to_week_offset_ - floor(msg->time + GPS_to_week_offset_))*1e9;
        imu_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(msg->time);
        uint64_t nsec = (msg->time - sec)*1e9;
        imu_time = ros::Time(sec, nsec) + IMU_offset_;
    }


    // Calculate an offset so we can sync IMU messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - imu_time;
        imu_time += IMU_offset_;
    }
    imu1_msg.header.stamp = imu2_msg.header.stamp = imu_time;
    imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

    imu1_msg.angular_velocity.x = msg->I[0].pqr[0];
    imu1_msg.angular_velocity.y = msg->I[0].pqr[1];
    imu1_msg.angular_velocity.z = msg->I[0].pqr[2];
    imu1_msg.linear_acceleration.x = msg->I[0].acc[0];
    imu1_msg.linear_acceleration.y = msg->I[0].acc[1];
    imu1_msg.linear_acceleration.z = msg->I[0].acc[2];

    imu2_msg.angular_velocity.x = msg->I[1].pqr[0];
    imu2_msg.angular_velocity.y = msg->I[1].pqr[1];
    imu2_msg.angular_velocity.z = msg->I[1].pqr[2];
    imu2_msg.linear_acceleration.x = msg->I[1].acc[0];
    imu2_msg.linear_acceleration.y = msg->I[1].acc[1];
    imu2_msg.linear_acceleration.z = msg->I[1].acc[2];

    if (IMU_.stream_on)
    {
        IMU_.pub.publish(imu1_msg);
        IMU_.pub2.publish(imu2_msg);
    }
}


void InertialSenseROS::GPS_callback(const gps_nav_t * const msg)
{
    uint64_t seconds = GPS_UTC_OFFSET + msg->week*7*24*3600 + floor(msg->timeOfWeekMs/1e3);
    uint64_t nsec = (msg->timeOfWeekMs/1e3 - floor(msg->timeOfWeekMs/1e3))*1e9;
    GPS_week_seconds = msg->week*7*24*3600;
    gps_msg.header.stamp = ros::Time(seconds, nsec);
    gps_msg.fix_type = msg->status & GPS_STATUS_FIX_FLAGS_MASK;
    gps_msg.header.frame_id =frame_id_;
    gps_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg.cno = msg->cnoMean;
    gps_msg.latitude = msg->lla[0];
    gps_msg.longitude = msg->lla[1];
    gps_msg.altitude = msg->lla[2];
    gps_msg.hMSL = msg->hMSL;
    gps_msg.hAcc = msg->hAcc;
    gps_msg.vAcc = msg->vAcc;
    gps_msg.pDop = msg->pDop;
    gps_msg.linear_velocity.x = msg->velNed[0];
    gps_msg.linear_velocity.y = msg->velNed[1];
    gps_msg.linear_velocity.z = msg->velNed[2];
    gps_msg.ground_speed_2d = pow(msg->velNed[0]*msg->velNed[0] + msg->velNed[1]*msg->velNed[1], 0.5);
    gps_msg.ground_speed_3d = pow(msg->velNed[0]*msg->velNed[0] + msg->velNed[1]*msg->velNed[1] + msg->velNed[2]*msg->velNed[2], 0.5);
    gps_msg.course = 0;//
    gps_msg.cAcc = 0;// d_.gps.vel.cAcc;
    gps_msg.messages_per_second = -1; //msg->r;
    GPS_.pub.publish(gps_msg);

    if (!got_GPS_fix_)
    {
        if (msg->status & GPS_STATUS_FIX_STATUS_3D_FIX)
        {
            got_GPS_fix_ = true;
        }
        GPS_to_week_offset_ = msg->towOffset;
    }
}

void InertialSenseROS::GPS_Info_callback(const gps_sat_t* const msg)
{
    uint64_t seconds = GPS_UTC_OFFSET + GPS_week_seconds + floor(msg->timeOfWeekMs/1e3);
    uint64_t nsec = (msg->timeOfWeekMs - floor(msg->timeOfWeekMs))*1e6;
    //  ROS_INFO("dsec = %d dnsec = %d", ros::Time::now().sec - seconds, ros::Time::now().nsec - nsec);
    gps_info_msg.header.stamp = ros::Time(seconds, nsec);
    gps_info_msg.header.frame_id = frame_id_;
    gps_info_msg.num_sats = msg->numSats;
    for (int i = 0; i < 50; i++)
    {
        gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
        gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
    }
    GPS_info_.pub.publish(gps_info_msg);
}


void InertialSenseROS::mag_callback(const magnetometer_t* const msg, int mag_number)
{
    sensor_msgs::MagneticField mag_msg;
    ros::Time mag_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp mag messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(msg->time + GPS_to_week_offset_);
        uint64_t nsec = (msg->time + GPS_to_week_offset_ - floor(msg->time + GPS_to_week_offset_))*1e9;
        mag_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(msg->time);
        uint64_t nsec = (msg->time - sec)*1e9;
        mag_time = ros::Time(sec, nsec) + IMU_offset_;
    }


    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - mag_time;
        mag_time += IMU_offset_;
    }

    mag_msg.header.stamp = mag_time;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];

    if(mag_number == 1)
    {
        mag_.pub.publish(mag_msg);
    }
    else
    {
        mag_.pub2.publish(mag_msg);
    }
}

void InertialSenseROS::baro_callback(const barometer_t * const msg)
{
    sensor_msgs::FluidPressure baro_msg;
    ros::Time baro_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp baro messagesgit@github.com:inertialsense/inertial_sense_ros.git
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(msg->time + GPS_to_week_offset_);
        uint64_t nsec = (msg->time + GPS_to_week_offset_ - floor(msg->time + GPS_to_week_offset_))*1e9;
        baro_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(msg->time);
        uint64_t nsec = (msg->time - sec)*1e9;
        baro_time = ros::Time(sec, nsec) + IMU_offset_;
    }
    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - baro_time;
        baro_time += IMU_offset_;
    }

    baro_msg.header.stamp = baro_time;
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;

    baro_.pub.publish(baro_msg);
}

void InertialSenseROS::dtheta_vel_callback(const dual_imu_dtheta_dvel_t * const msg)
{
    inertial_sense::DThetaVel dthetavel_msg;

    ros::Time imu_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp imu messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(msg->time + GPS_to_week_offset_);
        uint64_t nsec = (msg->time + GPS_to_week_offset_ - floor(msg->time + GPS_to_week_offset_))*1e9;
        imu_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(msg->time);
        uint64_t nsec = (msg->time - sec)*1e9;
        imu_time = ros::Time(sec, nsec) + IMU_offset_;
    }
    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - imu_time;
        imu_time += IMU_offset_;
    }

    dthetavel_msg.header.stamp = imu_time;
    dthetavel_msg.header.frame_id = frame_id_;
    dthetavel_msg.theta.x = msg->theta1[0];
    dthetavel_msg.theta.y = msg->theta1[1];
    dthetavel_msg.theta.z = msg->theta1[2];

    dthetavel_msg.vel.x = msg->vel1[0];
    dthetavel_msg.vel.y = msg->vel1[1];
    dthetavel_msg.vel.z = msg->vel1[2];

    dthetavel_msg.dt = msg->dt;

    dt_vel_.pub.publish(dthetavel_msg);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "inertial_sense_node");
    InertialSenseROS thing;
    ros::spin();
    return 0;
}

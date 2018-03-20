# inertial_sense

A ROS wrapper for the InertialSense uINS2 GPS-INS sensor

## NOTICE:

This node is in beta.  Release is coming in the next few weeks.  Until then, you will need to load the latest beta firmware on your uINS

(beta firmware link)[https://github.com/inertialsense/inertialsense_serial_protocol/releases/tag/v1.2-beta-0]

## Functionality
- INS full odometry streaming
- Dual IMU streaming
- Full GPS data streaming
- Magnetomter streaming
- Barometer streaming
- Coning and Sculling integral streaming
- Timestamping uses GPS timestamps when available and syncs sensor messages with ROS time if unavailable.
- Flash configuration via parameters
- Changing dynamic model via parameter


## Installation
This is a ROS package, with the InertialSenseSDK as a submodule, so just create a catkin workspace, clone this into the `src` folder, pull down the submodule and build

``` bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://inertialsense/inertial_sense
cd inertial_sense
git submodule update --init --recursive
cd ../..
catkin_make
```

## Running the Node

```bash
rosrun inertial_sense inertial_sense_node
```

Make sure that you are a member of the `dailout` group, or you won't have access to the serial port.

For changing parameter values and topic remapping from the command line using `rosrun` refer to the [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments) page. For setting vector parameters, use the following syntax:

``` bash
rosparam set /inertial_sense_node/GPS_ref_lla "[40.25, -111.67, 1556.59]"
rosrun inertial_sense inertial_sense_node
```

For setting parameters and topic remappings from a launch file, refer to the [Roslaunch for Larger Projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) page, or the sample `launch/test.launch` file.

## Time Stamps

If GPS is available, all header timestamps are calculated with respect to the GPS clock but are translated into UNIX time to be consistent with the other topics in a ROS network.  If GPS is unvailable, then message headers are assigned a timestamp when they arrive at the computer with ROS time.  In an ideal setting, there should be no jump in timestamps when GPS is first acquired, because the timestamps should be identical, however, due to inaccuracies in system time, there will likely be a small adjustment to message timestamps when GPS is first acquired.


## Parameters
* `~port` (string, default: "/dev/ttyUSB0")
  - Serial port to connect to
* `~baud` (int, default: 3000000)
  - baudrate of serial communication
* `~frame_id` (string, default "body")
  - frame id of all measurements
* `~INS_rate` (int, default: 200)
  - The rate of odometry measurement streaming (Hz)
* `~IMU_rate`(int, default: 100)
   - The rate of IMU measurement streaming (Hz)
* `~GPS_rate`(int, default: 0)
   - The rate of GPS message streaming (Hz)
* `~GPS_info_rate`(int, default: 0)
    - The rate of GPS message streaming (Hz)
* `~baro_rate` (int, default: 0)
    - The rate of barometer streaming (Hz)
* `~mag_rate` (int, default: 0)
    - The rate of magnetometer streaming (Hz)
* `~preint_imu_rate` (int, default: 0)
    - The rate of preintegrated coning and sculling integral message streaming
* `~INS_rpy` (vector(3), default: {0, 0, 0})
    - The roll, pitch, yaw rotation from the INS frame to the output frame
* `~INS_xyz` (vector(3), default: {0, 0, 0})
    - The NED translation vector between the INS frame and the output frame (wrt output frame)
* `~GPS_ang_xyz` (vector(3), default: {0, 0, 0})
    - The NED translation vector between the INS frame and the GPS antenna (wrt INS frame)
* `~GPS_ref_lla` (vector(3), default: {0, 0, 0})
    - The Reference longitude, latitude and altitude for NED calculation in degrees, degrees and meters
* `~inclination` (float, default: 1.14878541071)
    - The inclination of earth's magnetic field (radians)
* `~declination` (float, default: 0.20007290992)
    - The declination of earth's magnetic field (radians)
* `~dynamic_model` (int, default: 8)
    - Dynamic model used in internal filter of uINS.
       - 0 = portable
       - 2 = stationary
       - 3 = pedestrian
       - 4 = automotive
       - 5 = sea
       - 6 = airborne 1G
       - 7 = airborne 2G
       - 8 = airborne 4G
       - 9 = wrist
* `~ser1_baud_rate` (int, default: 115200)
    - baud rate for serial1 port used for external NMEA messaging (located on H6-5) [serial port hardware connections](http://docs.inertialsense.com/user-manual/Setup_Integration/hardware_integration/#pin-definition)
* `~NMEA_rate` (int, default: 0)
    - Rate to publish NMEA messages
* `~NMEA_configuration` (int, default: 0)
    - bitmask to enable NMEA messages (bitwise OR to enable multiple message streams).
      - GPGGA = 0x01
      - GPGLL = 0x02
      - GPGSA = 0x04
      - GPRMC = 0x08
* `~NMEA_ports` (int, default: 0)
    - bitmask to enable NMEA message on serial ports (bitwise OR to enable both ports) 
      - Ser0 (USB/H4-4)  = 0x01 
      - Ser1 (H6-5) = 0x02 

## Topics
- `ins/`(nav_msgs/Odometry)
    - full 12-DOF measurements from onboard estimator (NED frame)
- `imu1/`(sensor_msgs/Imu)
    - Raw Imu measurements from IMU1 (NED frame)
- `imu2/`(sensor_msgs/Imu)
    - Raw Imu measurements from IMU2 (NED frame)
- `gps/`(inertial_sense/GPS)
    - unfiltered GPS measurements from onboard GPS unit
- `gps/info`(inertial_sense/GPSInfo)
    - sattelite information and carrier noise ratio array for each sattelite
- `mag1` (sensor_msgs/MagneticField)
    - Raw magnetic field measurement from magnetometer 1
- `mag2` (sensor_msgs/MagneticField)
    - Raw magnetic field measurement from magnetometer 2
- `baro` (sensor_msgs/FluidPressure)
    - Raw barometer measurements in kPa
- `preint_imu` (inertial_sense/DThetaVel)
    - preintegrated coning and sculling integrals of IMU measurements

## Services
- `single_axis_mag_cal` (std_srvs/Trigger)
  - Put INS into single axis magnetometer calibration mode.  This is typically used if the uINS is rigidly mounted to a heavy vehicle, such as a car. After this call, the uINS must perform a single orbit around one axis (i.g. drive in a circle) to calibrate the mag [more info](http://docs.inertialsense.com/user-manual/Setup_Integration/magnetometer_calibration/)
- `multi_axis_mag_cal` (std_srvs/Trigger)
  - Put INS into multi axis magnetometer calibration mode.  This is typically used if the uINS is not mounted to a vehicle, or a lightweight vehicle such as a drone.  Simply rotate the uINS around all axes until the light on the uINS turns blue [more info](http://docs.inertialsense.com/user-manual/Setup_Integration/magnetometer_calibration/)
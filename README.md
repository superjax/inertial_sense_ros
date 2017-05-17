# inertial_sense

A ROS wrapper for the InertialSense uINS2 GPS-INS sensor

## Functionality 
- INS full odometry streaming
- Dual IMU streaming
- Full GPS data streaming

## ToDo
- Fix timestamping to use GPS timestamps
- Magnetomter streaming
- Barometer streaming
- Coning and Sculling integral streaming
- Adjusting rate of streaming on-the-fly with dynamic reconfigure

## Installation
This is a ROS package, with the InertialSenseSDK as a submodule, so just create a catkin workspace, clone this into the `src` folder, pull down the submodule and build

``` bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://superjax/inertial_sense
cd inertial_sense
git submodule update --init --recursive
cd ../..
catkin_make
```

## Running the Node

```bash
rosrun inertial_sense inertial_sense_node
```

For changing parameter values and topic remapping from the command line using `rosrun` refer to the [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments) page.

For setting parameters and topic remappings from a launch file, refer to the [Roslaunch for Larger Projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) page.

## Parameters
- `~port` (string, default: "/dev/ttyUSB0")
    - Serial port to connect to
- `~baud` (int, default: 3000000)
    - baudrate of serial communication 
- `~frame_id` (string, default "body") 
    - frame id of all measurements 
- `~sINS` (bool, default: true)
    - Whether to stream the full 12-DOF odometry measurement
- `~sINS_rate` (int, default: 100)
    - The rate of odometry measurement streaming (Hz)
- `~sIMU`(bool, default: true)
    - Whether to stream IMU measurements
- `~sIMU_rate`(int, default: 100) 
    - The rate of IMU measurement streaming (Hz)
- `~sGPS`(bool, default: true) 
    - If true, the node will stream GPS measurements
- `~sGPS_rate`(int, default: 10) 
    - The rate of GPS message streaming (Hz)

## Topics
- `imu1/`(sensor_msgs/Imu) 
    - Imu measurements from IMU1 (NED frame)
- `imu2/`(sensor_msgs/Imu) 
    - Imu measurements from IMU2 (NED frame)
- `ins/`(nav_msgs/Odometry) 
    - full 12-DOF measurements from onboard estimator (NED frame)
- `gps/`(inertial_sense/GPS) 
    - full GPS measurement from onbaord GPS

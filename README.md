# Livox ROS2 Driver([览沃ROS2驱动程序中文说明](https://github.com/Livox-SDK/livox_ros2_driver/blob/master/README_CN.md))

The Livox ROS2 driver is a driver package based on ROS2, specifically used to connect LiDAR products produced by Livox.
## 0. Version and Release History

### 0.1 Current Version

[v0.0.1-beta](https://github.com/Livox-SDK/livox_ros2_driver/releases/tag/v0.0.1-beta)

### 0.2 Release History

[Release History](https://github.com/Livox-SDK/livox_ros2_driver/releases/tag/v0.0.1-beta)

## 1. Install dependencies

Before running Livox ROS2 driver under ubuntu18.04, ROS2 (dashing, ubuntu18.04), colcon build tool and Livox-SDK must be installed.

### 1.1 ROS2 installation

For ROS2 installation, please refer to the ROS2 installation guide :

[ROS2 installation guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

***Note :***

（1）Be sure to install the desktop version of ROS2 (ros-dashing-desktop);

（2）After installing ROS2 dashing, please follow the installation guide to configure the system environment;

### 1.2 install colcon

Please refer to the following link for colcon installation：

[colcon installation](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)

## 2. Get and build livox_ros2_driver

1. Get livox_ross_driver from GitHub :

   `git clone https://github.com/Livox-SDK/livox_ros2_driver.git ws_livox/src`

***Note :***

Be sure to use the above command to clone the code to the local, otherwise it will compile error due to the file path problem.

1. Use the following command to build livox_ros2_driver :

   ```bash
   cd ws_livox
   colcon build
   ```

***IMPORTANT :***

If you would like to compile livox_ros2_driver, be sure to delete/rename the "livox_sdk_static.a" file in the "/usr/local/lib/" directory. The livox_ros2_driver would build a shared library of [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) on its own.

1. Source the environment

   `source ./install/setup.sh`

## 3. Run livox_ros2_driver

### 3.1 Use the ROS2 launch file to load livox_ros2_driver

   Before using the launch file to load livox_ros2_driver, first enter the directory where the launch file is located, the command is as follows：

   `cd ./src/livox_ros2_driver/launch`

   The command format for loading livox_ros2_driver using the launch file is as follows：

   `ros2 launch [launch file]`

### 3.2 ROS2 launch load command example

   In lidar connection mode, the commands to load livox_ros2_driver and rviz2 are as follows :
   `ros2 launch livox_lidar_rviz_launch.py`

   In hub connection mode, the commands to load livox_ros2_driver and rviz2 are as follows :
   `ros2 launch livox_hub_rviz_launch.py`

### 3.3 ROS2 launch loads livox_ros2_driver precautions

1. The launch method of ros2 is completely different from that of ros1. The launch file of ros2 is actually a python script;

2. The ros2 driver no longer supports specifying the LiDAR device to be connected under the command line, and only supports configuring the corresponding LiDAR broadcast code and other parameters in the json configuration file;

3. When the connection status of the device specified in the configuration file is configured to enable connection (true), the livox_ros2_driver will only connect to the device specified in the configuration file;

4. When the connection status of the devices specified in the configuration file is all configured to prohibit connection (false), livox_ros2_driver will automatically connect all the devices that are scanned;

5. the json configuration file is in the "ws_livox/src/livox_ros2_driver/config" directory;

### 3.4 Livox LiDAR Broadcast code introduction

Each Livox LiDAR device has a unique broadcast code. The broadcast code consists of a 14-character serial number and an additional character (1, 2, or 3), for a total of 15 characters. The above serial number is located under the QR code of the LiDAR body shell (see the figure below). The broadcast code is used to specify the LiDAR device to be connected. The detailed format is as follows :

&ensp;&ensp;&ensp;&ensp;![Broadcast Code](images/broadcast_code.png)

***Note :***

X in the figure above corresponds to 1 in MID-100_Left/MID-40/Horizon/Tele products, 2 in MID-100_Middle, and 3 in MID-100_Right.

## 4. Launch file and livox_ros2_driver internal parameter configuration instructions

### 4.1 Launch file configuration instructions

All launch files of livox_ros2_driver are in the "ws_livox/src/livox_ros2_driver/launch" directory. Different launch files have different configuration parameter values and are used in different scenarios :

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| livox_lidar_rviz_launch.py   | Connect to Livox LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| livox_hub_rviz_launch.py     | Connect to Livox Hub device<br>Publish pointcloud2 format data<br>Autoload rviz |
| livox_lidar_launch.py        | Connect to Livox LiDAR device<br>Publish pointcloud2 format data |
| livox_hub_launch.py          | Connect to Livox LiDAR device<br>Publish pointcloud2 format data |
| livox_lidar_msg_launch.py    | Connect to Livox LiDAR device<br>Publish livox customized pointcloud data |
| livox_hub_msg_launch.py      | Connect to Livox Hub device<br>Publish livox customized pointcloud data |

### 4.2 Livox_ros2_driver internal main parameter configuration instructions

All internal parameters of Livox_ros2_driver are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. | 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTL) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library | 0       |

&ensp;&ensp;&ensp;&ensp;***livox_ros2_driver pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTL) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity         # the value is reflectivity, 0.0~255.0
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

2. Livox customized data package format, as follows :

```c
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (Not currently supported) :

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 5. Configure LiDAR parameters

In the "ws_livox/src/livox_ros2_driver/launch" path, there are two json files, livox_hub_config.json and livox_lidar_config.json.

1. When connecting directly to LiDAR, use the livox_lidar_config.json file to configure LiDAR parameters. Examples of file contents are as follows :

```json
{
   "lidar_config": [
      {
         "broadcast_code": "0TFDG3B006H2Z11",
         "enable_connect": true,
         "enable_fan": true,
         "return_mode": 0,
         "coordinate": 0,
         "imu_rate": 1,
         "extrinsic_parameter_source": 0
      }
   ]
}
```

&ensp;&ensp;&ensp;&ensp;The parameter attributes in the above json file are described in the following table :

LiDAR configuration parameter
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| broadcast_code             | String  | LiDAR broadcast code, 15 characters, consisting of a 14-character length serial number plus a character-length additional code | 0TFDG3B006H2Z11 |
| enable_connect             | Boolean | Whether to connect to this LiDAR<br>true -- Connect this LiDAR<br>false --Do not connect this LiDAR | false           |
| return_mode                | Int     | return mode<br>0 -- First single return mode<br>1 -- Strongest single return mode<br>2 -- Dual return mode | 0               |
| coordinate                 | Int     | Coordinate<br>0 -- Cartesian<br>1 -- Spherical               | 0               |
| imu_rate                   | Int     | Push frequency of IMU sensor data<br>0 -- stop push<br>1 -- 200 Hz<br>Others -- undefined, it will cause unpredictable behavior<br>Currently only Horizon supports this, MID serials do not support it | 0               |
| extrinsic_parameter_source | Int     | Whether to enable extrinsic parameter automatic compensation<br>0 -- Disable automatic compensation of LiDAR external reference<br>1 -- Automatic compensation of LiDAR external reference | 0               |

***Note :***

When connecting multiple LiDAR, if you want to use the external parameter automatic compensation function, you must first use the livox viewer to calibrate the external parameters and save them to LiDAR.

1. When connecting to the Hub, use livox_hub_config.json to configure the parameters of the Hub and LiDAR. Examples of file contents are as follows :

```json
{
   "hub_config": {
      "broadcast_code": "13UUG1R00400170",
      "enable_connect": true,
      "coordinate": 0
   },
   "lidar_config": [
      {
         "broadcast_code": "0TFDG3B006H2Z11",
         "return_mode": 0,
         "imu_rate": 1
      }
   ]
}
```

&ensp;&ensp;&ensp;&ensp;The main difference between the content of Hub json configuration file and the content of the LiDAR json configuration file is that the Hub configuration item "hub_config" is added, and the related configuration content of the Hub is shown in the following table :

HUB configuration parameter
| Parameter      | Type    | Description                                                  | Default         |
| -------------- | ------- | ------------------------------------------------------------ | --------------- |
| broadcast_code | String  | HUB broadcast code, 15 characters, consisting of a 14-character length serial number plus a character-length additional code | 13UUG1R00400170 |
| enable_connect | Boolean | Whether to connect to this Hub<br>true -- Connecting to this Hub means that all LiDAR data connected to this Hub will be received<br>false -- Prohibition of connection to this Hub means that all LiDAR data connected to this Hub will not be received | false           |
| coordinate     | Int     | Coordinate<br>0 -- Cartesian<br>1 -- Spherical             | 0               |

***Note :***

(1) The configuration parameters enable_connect and coordinate in the Hub configuration item "hub_config" are global and control the behavior of all LiDARs. Therefore, the LiDAR related configuration in the Hub json configuration file does not include these two contents.

(2) The Hub itself supports compensation of LiDAR external parameters, and does not require livox_ros2_driver to compensate.

## 6. livox_ros2_driver timestamp synchronization function

### 6.1 Hardware requirements

Prepare a GPS device to ensure that the GPS can output UTC time information in GPRMC/GNRMC format through the serial port or USB virtual serial port, and support PPS signal output; then connect the GPS serial port to the host running livox_ros2_driver, and connect the GPS PPS signal line to LiDAR. For detailed connection instructions and more introduction to time stamp synchronization, please refer to the following links:

[Timestamp synchronization](https://github.com/Livox-SDK/Livox-SDK/wiki/Timestamp-Synchronization)

***Note :***

(1) The time stamp synchronization function of livox_ros2_driver is based on the LidarSetUtcSyncTime interface of Livox-SDK, and only supports GPS synchronization, which is one of many synchronization methods of livox devices.

(2) Be sure to set the output frequency of GPRMC/GNRMC time information of GPS to 1Hz, other frequencies are not recommended.

(3) Examples of GPRMC/GNRMC format strings are as follows :

```bash
$GNRMC,143909.00,A,5107.0020216,N,11402.3294835,W,0.036,348.3,210307,0.0,E,A*31
$GNRMC,021225.00,A,3016.60101,N,12007.84214,E,0.011,,260420,,,A*67
$GPRMC,010101.130,A,3606.6834,N,12021.7778,E,0.0,238.3,010807,,,A*6C
$GPRMC,092927.000,A,2235.9058,N,11400.0518,E,0.000,74.11,151216,,D*49
$GPRMC,190430,A,4812.3038,S,07330.7690,W,3.7,3.8,090210,13.7,E,D*26
```

### 6.2 Enable timestamp synchronization

livox_ros2_driver only supports the timestamp synchronization function when connected to LiDAR. The timestamp related configuration item timesync_config is in the livox_lidar_config.json file. The detailed configuration content is shown in the table below :

Timestamp synchronization function configuration instructions
| Parameter        | Type     | Description                                                  | Default        |
| ---------------- | -------- | ------------------------------------------------------------ | -------------- |
| enable_timesync  | Boolean  | Whether to enable the timestamp synchronization <br>true -- Enable timestamp synchronization<br>false -- Disable timestamp synchronization | false          |
| device_name      | String | Name of the serial device to be connected, take "/dev/ttyUSB0" as an example, indicating that the device sending timestamp information to livox_ros2_driver is ttyUSB0 | "/dev/ttyUSB0" |
| comm_device_type | Int      | Type of device sending timestamp information<br>0 -- Serial port or USB virtual serial port device<br>other -- not support | 0              |
| baudrate_index   | Int      | Baud rate of serial device<br>0 -- 2400 <br>1 -- 4800 <br>2 -- 9600 <br>3 -- 19200 <br>4 -- 38400 <br>5 -- 57600 <br>6 -- 115200 <br>7 -- 230400 <br>8 -- 460800 <br>9 -- 500000 <br>10 -- 576000 <br>11 -- 921600 | 2              |
| parity_index     | Int      | parity type<br>0 -- 8bits data without parity<br>1 -- 7bits data 1bit even parity<br>2 -- 7bits data 1bit odd parity<br>3 -- 7bits data 1bit 0, without parity | 0              |

## 7. Support

You can get support from Livox with the following methods :

* Send email to cs@livoxtech.com with a clear description of your problem and your setup
* Report issue on github

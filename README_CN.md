# 览沃 ROS2 驱动程序（ [livox_ros2_driver English README](https://github.com/Livox-SDK/livox_ros2_driver/) ）

览沃 ROS2 驱动程序是基于 ROS2 的驱动程序包，专门用于连接览沃生产的 LiDAR 产品。该驱动程序目前仅推荐在 ubuntu18.04 下运行，对应的 ROS2 版本是 dashing， 暂时不支持 ROS2 其他版本。

## 0. 版本和发布记录

### 0.1 当前版本

v0.0.1beta

### 0.2 发布记录

[发布记录](https://github.com/Livox-SDK/livox_ros2_driver/releases)

## 1. 安装依赖

在 ubuntu18.04 下运行览沃 ROS2 驱动程序之前，必须安装 ROS2(dashing，ubuntu18.04)，colcon 构建工具和 Livox-SDK 。

### 1.1 ROS2 环境安装

ROS2 环境安装请参考 ROS2 安装指南：

[ROS2 安装指南](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

***说明：***

（1）务必安装 ROS2 桌面版 (ros-dashing-desktop)；

（2）安装完 ROS2 dashing 后， 请遵照安装指南配置系统环境；

### 1.2 colcon 构建工具安装

colcon 安装请参考如下链接：

[colcon 安装](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)


## 2. 获取并构建览沃 ROS2 驱动源代码包

1. 从览沃 GitHub 获取览沃 ROS2 驱动程序

   `git clone https://github.com/Livox-SDK/livox_ros2_driver.git ws_livox/src`

***说明：***

务必使用上面的命令克隆代码到本地，否则会因为文件路径的问题而编译出错

1. 参照如下命令，构建览沃 ROS2 驱动程序

   ```bash
   cd ws_livox
   colcon build
   ```

***重要：***

若需要构建览沃 ROS2 驱动程序，务必将 "/usr/local/lib/" 目录下的 "livox_sdk_static.a" 文件删除或改名，再执行如上构建步骤。览沃 ROS2 驱动程序自身将编译 [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) 为动态库。

1. 使用如下命令更新当前 ROS2 包环境

   `source ./install/setup.sh`

## 3. 运行览沃 ROS2 驱动程序

### 3.1 使用 ROS2 launch 文件加载览沃 ROS2 驱动

   使用 launch 文件加载 livox_ros2_driver 之前，先进入到 launch 文件所在目录，命令如下：

   `cd ./src/livox_ros2_driver/launch`

   使用 launch 文件加载 livox_ros2_driver 的命令格式如下：

   `ros2 launch [launch file]`

### 3.2 ROS2 launch 命令示例

   lidar 连接模式下，加载 livox_ros2_driver 和 rviz2 的命令如下：
   `ros2 launch livox_lidar_rviz_launch.py`

   hub 连接模式下，加载 livox_ros2_driver 和 rviz2 的命令如下：
   `ros2 launch livox_hub_rviz_launch.py`

### 3.3 ROS2 使用 launch 加载 livox_ros2_driver 说明

1. ros2 的 launch 加载方式与 ros1 截然不同，ros2 的 launch 文件实际上是 python 脚本；

2. ros2 driver 不再支持在命令行下指定将要连接的 LiDAR 设备，只支持在 json 配置文件中配置相应的 LiDAR 广播码和其他参数；

3. 当配置文件中指定的设备连接状态配置为使能连接时 (true) ，览沃 ROS2 驱动程序只会连接该配置文件中指定的设备；

4. 当配置文件中指定的设备连接状态全部配置为禁止连接 (false) 时，览沃 ROS2 驱动程序会自动连接扫描到的所有设备；

5. 该配置文件位于 "ws_livox/src/livox_ros2_driver/config" 目录下；

### 3.4 Livox LiDAR 广播码说明

每台览沃 LiDAR 设备拥有一个唯一的广播码。广播码由14位字符长度的序列号和一个额外的字符组成（ 1、2或者 3），一共 15 位字符长度，上述序列号位于 LiDAR 机身外壳的二维码下面（见下图）。广播码被用来指定要连接的 LiDAR 设备，详细组成格式如下：

&ensp;&ensp;&ensp;&ensp;![Broadcast Code](images/broadcast_code.png)

***说明：***

上图中 X ，在 MID-100_Left/MID-40/Horizon/Tele 产品中对应为 1 ，在 MID-100_Middle 中对应为 2，在MID-100_Right 中对应为 3 。

## 4. Launch 文件与览沃 ROS2 驱动程序内部参数配置说明

### 4.1 Launch 配置文件描述

览沃驱动程序中所有的 launch 文件都位于 "ws_livox/src/livox_ros2_driver/launch" 路径下，不同的 launch 文件拥有不同的配置参数值， 应用在不同的场景中:

| launch 文件名             | 功能                                                         |
| ------------------------- | ------------------------------------------------------------ |
| livox_lidar_rviz_launch.py   | 连接览沃雷达设备<br>向外发布 pointcloud2 格式的点云数据<br>自动加载rviz |
| livox_hub_rviz_launch.py     | 连接览沃中心板设备<br>向外发布 pointcloud2 格式的点云数据<br>自动加载rviz |
| livox_lidar_launch.py        | 连接览沃雷达设备<br>向外发布 pointcloud2 格式的点云数据    |
| livox_hub_launch.py          | 连接览沃中心板设备<br>向外发布 pointcloud2 格式的点云数据  |
| livox_lidar_msg_launch.py    | 连接览沃雷达设备<br>向外发布览沃自定义点云数据             |
| livox_hub_msg_launch.py      | 连接览沃中心板设备<br/>向外发布览沃自定义点云数据           |

### 4.2 览沃 ROS2 驱动程序内部主要参数配置说明

览沃 ROS2 驱动程序中的所有内部参数都位于 launch 文件中，下面将对经常用到的三个参数进行详细说明:

| 参数名       | 详细说明                                                     | 默认值 |
| ------------ | ------------------------------------------------------------ | ------ |
| publish_freq | 设置点云发布频率 <br>浮点数据类型，推荐值 5.0，10.0，20.0，50.0 等。 | 10.0   |
| multi_topic  | LiDAR 设备是否拥有独立的 topic 发布点云数据<br>0 -- 所有 LiDAR 设备共同使用同一个 topic 发送点云数据<br>1 -- 每个 LiDAR 设备各自拥有独立的 topic 发布点云数据 | 0      |
| xfer_format  | 设置点云格式<br>0 -- 览沃 pointcloud2(PointXYZRTL) 点云格式<br>1 -- 览沃自定义点云数据格式<br>2 -- PCL库中标准 pointcloud2(pcl::PointXYZI) 点云格式 | 0      |

### 4.3 览沃 ROS2 驱动程序点云数据详细说明

1. 览沃 pointcloud2(PointXYZRTL) 点云格式，详细说明如下:

```c

float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8 tag               # livox tag
uint8 line              # laser number in lidar

```

2. 览沃自定义数据包格式，详细说明如下 :

```c
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;上述自定义数据包中的自定义点云（CustomＰoint）格式  :

   ```c

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar

   ```

3. PCL 库中标准 pointcloud2(pcl::PointXYZI) 点云格式（暂时不支持） :

&ensp;&ensp;&ensp;&ensp;请参考 PCL 库 point_types.hpp 文件中 the pcl::PointXYZI 数据结构。

## 5. 配置 LiDAR 参数

在 "ws_livox/src/livox_ros2_driver/config" 路径下, 有两个 json 配置文件，分别为  livox_hub_config.json 和 livox_lidar_config.json 。

1. 直接连接 LiDAR 时，使用 livox_lidar_config.json 来配置 LiDAR 参数，文件内容示例如下：

   ```json
   {
      "lidar_config": [
         {
            "broadcast_code": "0TFDG3B006H2Z11",
            "enable_connect": true,
            "return_mode": 0,
            "coordinate": 0,
            "imu_rate": 1,
            "extrinsic_parameter_source": 0
         }
       ]
   }
   ```

&ensp;&ensp;&ensp;&ensp;上面 json 文件中各参数属性说明如下表：

LiDAR 配置参数说明
| 属性                       | 类型   | 描述                                                         | 默认值          |
| :------------------------- | ------ | ------------------------------------------------------------ | --------------- |
| broadcast_code             | 字符串 | LiDAR 广播码，15位字符，由14位字符长度序列号加一个字符长度附加码组成 | 0TFDG3B006H2Z11 |
| enable_connect             | 布尔值 | 是否连接此 LiDAR<br>true -- 连接此 LiDAR<br>false -- 禁止连接此 LiDAR | false           |
| return_mode                | 整型   | 回波模式<br>0 -- 第一个回波模式<br>1 -- 最强回波模式<br>2 -- 双回波模式 | 0               |
| coordinate                 | 整型   | 原始点云数据的坐标轴类型<br>0 -- 直角坐标系<br>1 -- 球坐标系 | 0               |
| imu_rate                   | 整型   | IMU 传感器数据的推送频率<br>0 -- 关闭 IMU 传感器数据推送<br>1 --  以 200Hz 频率推送 IMU 传感器数据<br>其他值 -- 未定义，会导致不可预测的行为发生<br>目前只有 Horizon/Tele 支持此选项，MID 序列不支持 | 0               |
| extrinsic_parameter_source | 整型   | 是否使能外参自动补偿<br>0 -- 不补偿 LiDAR 外参<br>1 -- 自动补偿 LiDAR 外参<br> | 0               |

***说明：***

连接多个 LiDAR 时，如果要使用外参自动补偿功能，必需先使用 livox viewer 标定好外参并保存到 LiDAR 中；

1. 连接中心板时，使用 livox_hub_config.json 来配置中心板和 LiDAR 相关的参数，文件内容示例如下：

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

&ensp;&ensp;&ensp;&ensp;中心板 json 配置文件内容与 LiDAR 配置文件的主要区别在于，增加了中心板配置项 hub_config ，中心板相关的具体配置内容见下表：

HUB 配置参数说明
| 属性           | 类型   | 描述                                                         | 默认值          |
| -------------- | ------ | ------------------------------------------------------------ | --------------- |
| broadcast_code | 字符串 | HUB 广播码，15位字符，由14位字符长度的序列号加一个字符长度的附加码组成 | 13UUG1R00400170 |
| enable_connect | 布尔值 | 是否连接当前 Hub，<br>true -- 连接此 Hub，意味着所有与此中心板相连接的 LiDAR 数据都会被接收 <br>false -- 禁止连接此 Hub，意味着所有与此中心板相连接的 LiDAR 数据都不会被接收 | false           |
| coordinate     | 整型   | 原始点云数据的坐标轴类型<br>0 -- 直角坐标系<br>1 -- 球坐标系 | 0               |

***说明***

（1）中心板配置项 hub_config 中配置参数 enable_connect 和 coordinate 是全局性的，控制着所有 LiDAR 的行为，因此中心板 json 配置文件中 LiDAR 相关的配置不包括这两项内容。

（2）中心板自身支持补偿 LiDAR 外参，无需览沃 ROS2 驱动程序来补偿。

## 6. 览沃 ROS2 驱动程序的时间戳同步功能

### 6.1 硬件要求

准备一台 GPS 设备，确保此 GPS 能够通过串口或者 USB 虚拟串口输出 GPRMC/GNRMC 格式的 UTC 时间信息，同时支持 PPS 信号输出；将 GPS 串口连接到运行览沃驱动程序的主机，将 GPS 的 PPS 信号连接到 LiDAR 的 PPS 信号线，详细的连接说明以及更多时间戳同步方式介绍请参考如下链接：

[时间戳同步](https://github.com/Livox-SDK/Livox-SDK/wiki/Timestamp-Synchronization)

***说明***

（1）览沃 ROS2 驱动程序的时间戳同步功能是基于 Livox-SDK 的 LidarSetUtcSyncTime 接口实现，且只支持 GPS 同步，是览沃设备多种同步方式的一种；

（2）务必将 GPS 的 GPRMC/GNRMC 时间信息的输出频率设置为 1Hz，其他频率不推荐；

（3）GPRMC/GNRMC 格式字符串示例如下：

```bash

　　$GNRMC,143909.00,A,5107.0020216,N,11402.3294835,W,0.036,348.3,210307,0.0,E,A*31
　　$GNRMC,021225.00,A,3016.60101,N,12007.84214,E,0.011,,260420,,,A*67
　　$GPRMC,010101.130,A,3606.6834,N,12021.7778,E,0.0,238.3,010807,,,A*6C
　　$GPRMC,092927.000,A,2235.9058,N,11400.0518,E,0.000,74.11,151216,,D*49
　　$GPRMC,190430,A,4812.3038,S,07330.7690,W,3.7,3.8,090210,13.7,E,D*26

```

### 6.2 使能时间戳同步功能

览沃 ROS2 驱动程序只有在与 LiDAR 连接的时候才支持时间戳同步功能，时间戳相关的配置项 timesync_config 位于 livox_lidar_config.json 文件中，详细的配置内容见下表：

时间戳同步功能配置说明
| 属性             | 类型   | 描述                                                         | 默认值         |
| ---------------- | ------ | ------------------------------------------------------------ | -------------- |
| enable_timesync  | 布尔值 | 是否使能时间戳同步功能<br>true -- 使能时间戳同步功能<br>false -- 禁止时间戳同步功能 | false          |
| device_name      | 字符串 | 要连接的串口设备名称，以 "/dev/ttyUSB0" 为例，表示向览沃驱动程序发送时间戳信息的设备是 ttyUSB0 | "/dev/ttyUSB0" |
| comm_device_type | 整型   | 发送时间戳信息的设备类型<br>0 -- 串口或者USB虚拟串口设备<br>其他 -- 不支持 | 0              |
| baudrate_index   | 整型   | 串口设备的波特率型<br>0 -- 2400 波特率<br>1 -- 4800 波特率<br>2 -- 9600 波特率<br>3 -- 19200 波特率<br>4 -- 38400 波特率<br>5 -- 57600 波特率<br>6 -- 115200 波特率<br>7 -- 230400 波特率<br>8 -- 460800 波特率<br>9 -- 500000 波特率<br>10 -- 576000 波特率<br>11 -- 921600 波特率 | 2              |
| parity_index     | 整型   | 串口信号的奇偶校验类型<br>0 -- 8bit数据无校验位<br>1 -- 7bit数据1bit偶校验<br>2 -- 7bit数据1bit奇校验<br>3 -- 7bit数据1bit 0，无校验 | 0              |

## 7. 支持

你可以通过以下方式获取 Livox 的技术支持 :

* 发送邮件到 cs@livoxtech.com ，详细描述您遇到的问题和使用场景
* 提交此代码仓的 github issues

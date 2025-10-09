# Livox ROS2 Driver - Power Modes

This is a short README about the ability to switch the power modes of the Livox lidar using ROS2 service calls.

## Launching the Driver

To launch the Livox-driver, modify the ```broadcast_code``` in file [livox_lidar_config.json](livox_ros2_driver/config/livox_lidar_config.json) to match the lidar you want to use. compile this ROS2-package and launch the driver with the following command:

```bash
ros2 launch livox_ros2_driver livox_lidar_launch.py
``` 

If the lidar was before in the standby- or powersaving-mode. it should wake up and switch to state normal to output the pointcloud.

## Getting the current Powermode

To get the current power-mode use the following command:

```bash
ros2 service call /livox/lidar/get_mode livox_interfaces/srv/GetMode "{}"
``` 

The output should now look something like this:

```
requester: making request: livox_interfaces.srv.GetMode_Request()

response:
livox_interfaces.srv.GetMode_Response(mode=1, success=True)
```

If the value of success is ```true```, then the service-call was successful and the mode-variable contains the current mode of the Livox-lidar.

If mode=```1```, the lidar is in the normal mode, a value of ```2``` means it is in the powersaving-mode and a value of ```3``` equals the standby-mode.


## Setting the Powermode

To set the mode of the Livox-lidar use the following command:

```bash
ros2 service call /livox/lidar/set_mode livox_interfaces/srv/SetMode "set_mode: 1"
```

The value of ```1``` means just like before the normal-mode, a value of ```2``` the powersaving-mode and ```3``` equals the standby-mode.

The output should now look something like this:

```
requester: making request: livox_interfaces.srv.SetMode_Request(set_mode=1)

response:
livox_interfaces.srv.SetMode_Response(success=True)
```

If the value of success is ```true```, then the service-call was successful and the powermode should have been changed.


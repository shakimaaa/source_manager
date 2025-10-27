
# Source Manager

## Overview
`SourceManager` is a ROS 2 node designed to manage multiple localization sources (such as GPS and SLAM), and to switch between them at runtime based on health status or user requests.  
The node is also responsible for triggering a restart mechanism when SLAM encounters anomalies, and for maintaining smooth alignment of position and orientation to ensure continuity during source switching.

## RUN
### GPS
```
  ros2 run odom_generator odom_generator_node 
  ros2 run source_manager source_manager_node 
```
### SLAM
```
ros2 launch xvins xvins_estimator.launch.py 
ros2 run source_manager source_manager_node
```
## Sub & pub
### odom_generator
- Sub : `/mav/global_position_int` `/mav/imu/data`
- Pub : `/position/gps_odom`

### source_manager
- Sub : `/position/gps_odom` `/position/slam_odom` `/mav/imu/data_raw` 
- Pub : `/odometry_` `/imu_propagate_` `/path_`


## Services

### 1. Manually Switch Localization Source
- **Service Name**: `/switch_source`
- **Type**: `xion_msg/srv/SwitchSourceType`
- **Request fields**:
  ```text
  int32 target   # 0=UNINIT, 1=GPS, 2=SLAM
  ---
  bool success
  string message

#### commands
- **change to slam-source**
```
ros2 service call /switch_source xion_msg/srv/SwitchSourceType "{target: 1}"
```
- **change to gps-source**
```
ros2 service call /switch_source xion_msg/srv/SwitchSourceType "{target: 0}"
```

- **restart xvins**
```
ros2 service call /set_restart_requested std_srvs/srv/SetBool "{data: true}"
```

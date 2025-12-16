# Quick Start Guide - Livox Mid360 SLAM System

## Prerequisites

✅ All installation steps completed (see [INSTALLATION.md](docs/INSTALLATION.md))
✅ Livox Mid360 sensor connected via Ethernet
✅ Network configured (sensor and computer on same subnet)

## Network Setup

1. **Configure your computer's network:**
   ```bash
   # Set static IP (replace enp0s1 with your interface name)
   sudo ifconfig enp0s1 192.168.1.50 netmask 255.255.255.0
   ```

2. **Verify Mid360 connectivity:**
   ```bash
   ping 192.168.1.1XX  # Replace XX with your sensor's last two digits
   ```

## Running the System

### Option 1: Full System (Recommended for first test)

```bash
# Source the workspace
source ~/catkin_ws/devel/setup.bash

# Launch everything
roslaunch mid360_nav full_system.launch
```

This starts:
- Livox Mid360 driver
- FAST-LIO2 mapping
- RVIZ visualization

### Option 2: Driver Only (Test sensor connectivity)

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mid360_nav driver_only.launch
```

Check data flow:
```bash
# In another terminal
rostopic hz /livox/lidar  # Should show ~10Hz
rostopic hz /livox/imu    # Should show ~200Hz
```

### Option 3: Mapping with Rosbag

```bash
# Record data
rosbag record /livox/lidar /livox/imu -O test_data.bag

# Play back and map
rosbag play test_data.bag
roslaunch mid360_nav mapping_only.launch
```

## Verifying the System

### 1. Check Topics
```bash
rostopic list
```
Expected topics:
- `/livox/lidar` - Raw LiDAR data
- `/livox/imu` - IMU data
- `/cloud_registered` - SLAM output
- `/Odometry` - Pose estimation
- `/path` - Trajectory

### 2. Check Data Rates
```bash
rostopic hz /livox/lidar     # ~10Hz
rostopic hz /cloud_registered # ~10Hz
```

### 3. Visualize in RVIZ
- **PointCloud2** displays show point clouds
- **Path** shows the trajectory
- **TF** shows coordinate frames

## Basic Operation

### Start Mapping
1. Launch the full system
2. Wait for initialization (~5 seconds)
3. Move the sensor slowly (or walk with it)
4. Observe the map building in RVIZ

### Stop and Save Map
```bash
# Kill the nodes (Ctrl+C)
# Map is automatically saved if pcd_save_en: true
```

### Record Test Data
```bash
# Record all topics
rosbag record -a -O my_test.bag

# Record specific topics
rosbag record /livox/lidar /livox/imu /Odometry -O slam_data.bag
```

## Common Commands

```bash
# Source workspace (add to ~/.bashrc for convenience)
source ~/catkin_ws/devel/setup.bash

# List ROS packages
rospack list | grep -E "fast_lio|livox"

# Check node status
rosnode list
rosnode info /laserMapping

# Monitor CPU/memory
htop

# View log output
roscd fast_lio
cat ~/.ros/log/latest/laserMapping-*.log
```

## Troubleshooting

### No LiDAR data (`/livox/lidar` not publishing)
1. Check network connection: `ping 192.168.1.1XX`
2. Check driver config: `~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json`
3. Restart driver: `rosnode kill /livox_lidar_publisher2`

### FAST-LIO not running
1. Check if topics exist: `rostopic list | grep livox`
2. Verify config file: `rosparam get /common/lid_topic`
3. Check logs: `tail -f ~/.ros/log/latest/laserMapping-*.log`

### Poor mapping quality
1. Reduce speed - move slower
2. Ensure good lighting (for feature tracking)
3. Adjust `blind` parameter in config
4. Check IMU calibration

### High CPU usage
1. Increase `filter_size_surf` to 0.6
2. Reduce `cube_side_length` to 100
3. Close unnecessary applications

## Next Steps

- [ ] Test in indoor environment
- [ ] Test in outdoor environment
- [ ] Record sample datasets
- [ ] Tune parameters for your use case
- [ ] Calibrate IMU-LiDAR extrinsics (if needed)

See [CONFIGURATION.md](docs/CONFIGURATION.md) for detailed parameter tuning.

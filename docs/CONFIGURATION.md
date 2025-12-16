# Configuration Guide for Mid360 SLAM System

## Configuration Files

### 1. FAST-LIO2 Configuration (`config/mid360_config.yaml`)

This file contains all the SLAM algorithm parameters.

#### Key Parameters to Tune:

**Preprocessing:**
```yaml
blind: 0.5  # Minimum detection distance
```
- **Indoor**: 0.3 - 0.5m
- **Outdoor**: 0.5 - 1.0m
- Filters out points too close to the sensor

**Mapping Quality:**
```yaml
filter_size_surf: 0.5  # Voxel filter size for surface points
filter_size_map: 0.5   # Voxel filter size for map
```
- **Higher quality (slower)**: 0.3 - 0.4m
- **Balanced**: 0.5m (default)
- **Faster (lower quality)**: 0.6 - 0.8m

**Map Size:**
```yaml
cube_side_length: 200  # Local map size in meters
```
- **Small environments**: 50 - 100m
- **Medium environments**: 100 - 200m
- **Large outdoor**: 200 - 500m

**IMU Noise Parameters:**
```yaml
acc_cov: 0.1      # Accelerometer noise
gyr_cov: 0.1      # Gyroscope noise
```
- Increase if IMU is noisy (jerky motion)
- Decrease for smoother, more accurate IMU

**Extrinsic Calibration:**
```yaml
extrinsic_T: [ -0.011, -0.02329, 0.04412 ]  # Translation
extrinsic_R: [ 1, 0, 0, 0, 1, 0, 0, 0, 1]   # Rotation
```
- Use default for Mid360 (IMU is inside)
- Set `extrinsic_est_en: true` for automatic calibration
- Manual calibration recommended for best accuracy

### 2. Livox Driver Configuration

Located at: `~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json`

**Default network settings:**
- Mid360 IP: Usually `192.168.1.1XX`
- Host IP: Set your computer to `192.168.1.50` (static)

To configure:
```bash
sudo nano ~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json
```

## Launch File Arguments

### Full System Launch
```bash
roslaunch mid360_nav full_system.launch rviz:=true
```

Arguments:
- `rviz:=true/false` - Enable/disable RVIZ visualization
- `publish_freq:=10.0` - LiDAR data rate (Hz)

### Driver Only
```bash
roslaunch mid360_nav driver_only.launch
```
Use for testing sensor connectivity.

### Mapping Only
```bash
roslaunch mid360_nav mapping_only.launch
```
Use with pre-recorded rosbag files.

## Performance Tuning

### For Real-Time Performance:
1. Increase filter sizes: `filter_size_surf: 0.6`
2. Reduce map size: `cube_side_length: 100`
3. Lower point filter: `point_filter_num: 5`

### For High Accuracy:
1. Decrease filter sizes: `filter_size_surf: 0.3`
2. Increase iterations: `max_iteration: 5`
3. Enable path: `path_en: true`

### For Large-Scale Mapping:
1. Increase map size: `cube_side_length: 500`
2. Enable PCD saving: `pcd_save_en: true`
3. Increase detection range: `det_range: 150.0`

## Common Issues and Solutions

### Issue: High CPU usage
**Solution:**
- Increase `filter_size_surf` to 0.6 or 0.7
- Reduce `cube_side_length`
- Lower `point_filter_num` to 5 or more

### Issue: Drift in odometry
**Solution:**
- Enable extrinsic estimation: `extrinsic_est_en: true`
- Adjust IMU noise: increase `acc_cov` and `gyr_cov`
- Calibrate IMU-LiDAR extrinsics manually

### Issue: Poor map quality
**Solution:**
- Decrease filter sizes to 0.3-0.4m
- Increase `max_iteration` to 4 or 5
- Check sensor mounting (vibrations?)

### Issue: Gaps in point cloud
**Solution:**
- Adjust `blind` parameter
- Check Mid360 connection and power
- Verify topic publication: `rostopic hz /livox/lidar`

## Coordinate Frames

- `map` / `camera_init` - Global reference frame
- `body` - Sensor body frame (IMU frame)
- `livox_frame` - LiDAR frame

Transform tree: `map` → `camera_init` → `body` → `livox_frame`

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Raw LiDAR data |
| `/livox/imu` | `sensor_msgs/Imu` | IMU data |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Registered map |
| `/Odometry` | `nav_msgs/Odometry` | Pose estimate |
| `/path` | `nav_msgs/Path` | Trajectory |

## Monitoring Performance

```bash
# Check topic rates
rostopic hz /livox/lidar
rostopic hz /cloud_registered

# Monitor resource usage
htop

# View TF tree
rosrun tf view_frames
evince frames.pdf

# Echo odometry
rostopic echo /Odometry
```

## Next Steps After Configuration

1. Test with live sensor
2. Record test rosbags
3. Tune parameters for your environment
4. Calibrate extrinsics if needed
5. Test in various scenarios (indoor/outdoor, fast/slow motion)

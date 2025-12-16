# FAST-LIO2 Implementation Plan for Livox Mid360

## Project Overview
Implement a complete ROS-based SLAM system using FAST-LIO2 (Fast LiDAR-Inertial Odometry) with Livox Mid360 sensor support and RVIZ visualization.

## Architecture
```
Livox Mid360 → Livox ROS Driver2 → FAST-LIO2 → RVIZ Visualization
                                      ↓
                                   Point Cloud Map
                                   Odometry/Pose
```

---

## Step-by-Step Implementation Plan

### Phase 1: Environment Setup

#### Step 1: ROS Workspace Setup
- [ ] Create catkin workspace structure
- [ ] Initialize workspace with `catkin_make` or `catkin build`
- [ ] Source workspace in `.bashrc`
- [ ] Verify ROS installation (ROS Noetic recommended for Ubuntu 20.04)

**Commands:**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### Step 2: Install System Dependencies
- [ ] Install essential build tools
- [ ] Install PCL (Point Cloud Library) 1.8 or higher
- [ ] Install Eigen3 library
- [ ] Install Ceres Solver (for optimization)
- [ ] Install other ROS dependencies

**Required packages:**
```bash
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-message-filters \
    ros-noetic-image-transport \
    ros-noetic-pcl-ros \
    libpcl-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev
```

---

### Phase 2: Livox SDK and Driver Installation

#### Step 3: Install Livox SDK2
- [ ] Clone Livox SDK2 repository
- [ ] Build and install SDK
- [ ] Verify installation

**Key points:**
- SDK2 is required for Mid360 (different from SDK1 for older sensors)
- Provides low-level communication with Mid360

```bash
cd ~/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

#### Step 4: Install Livox ROS Driver2
- [ ] Clone livox_ros_driver2 into workspace
- [ ] Configure for Mid360 sensor
- [ ] Build driver package
- [ ] Test driver with sensor

```bash
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
catkin_make
```

**Configuration:**
- Edit `config/MID360_config.json` for sensor IP and settings
- Default topic: `/livox/lidar` (CustomMsg format)

---

### Phase 3: FAST-LIO2 Installation

#### Step 5: Install FAST-LIO2 Dependencies
- [ ] Install ikd-Tree (incremental kd-tree for FAST-LIO2)
- [ ] Verify all dependencies are met

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/ikd-Tree.git
```

#### Step 6: Clone and Build FAST-LIO2
- [ ] Clone FAST-LIO2 repository
- [ ] Review and install any missing dependencies
- [ ] Build the package
- [ ] Verify successful compilation

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd ..
catkin_make
source devel/setup.bash
```

---

### Phase 4: Configuration

#### Step 7: Configure FAST-LIO2 for Mid360
- [ ] Create custom config file for Mid360
- [ ] Set correct topic names matching Livox driver output
- [ ] Configure IMU parameters (if using external IMU)
- [ ] Set extrinsic calibration (LiDAR to IMU/base_link)
- [ ] Tune FAST-LIO2 parameters for indoor/outdoor use

**Key configuration file:** `FAST_LIO/config/mid360.yaml`

**Important parameters:**
```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"  # Mid360 has built-in IMU
    
preprocess:
    lidar_type: 1  # 1 for Livox Avia/Mid360
    scan_line: 4
    blind: 0.5  # Minimum distance
    
mapping:
    filter_size_surf: 0.5
    filter_size_map: 0.5
    cube_side_length: 200
```

#### Step 8: Create Launch Files
- [ ] Create master launch file to start entire system
- [ ] Create separate launch files for:
  - Livox driver only
  - FAST-LIO2 with driver
  - RVIZ visualization
- [ ] Add parameter configurations

**Launch file structure:**
```
launch/
├── livox_mid360.launch      # Start Livox driver
├── fast_lio_mid360.launch   # FAST-LIO2 + driver
└── visualization.launch      # RVIZ config
```

---

### Phase 5: Visualization Setup

#### Step 9: Configure RVIZ
- [ ] Create RVIZ configuration file
- [ ] Add PointCloud2 display for live scan
- [ ] Add PointCloud2 display for map
- [ ] Add Odometry/Path display for trajectory
- [ ] Add TF display for coordinate frames
- [ ] Configure color schemes and point sizes
- [ ] Save RVIZ config

**RVIZ Displays to add:**
1. **PointCloud2** - Current scan (`/livox/lidar`)
2. **PointCloud2** - Map (`/cloud_registered`)
3. **Path** - Odometry path (`/path`)
4. **Odometry** - Pose (`/Odometry`)
5. **TF** - All transforms
6. **Axes** - Coordinate frames

#### Step 10: Create RVIZ Config File
- [ ] Set fixed frame to `camera_init` or `world`
- [ ] Configure appropriate point cloud size and color
- [ ] Set up camera views (top-down, perspective)
- [ ] Save configuration to `config/rviz_config.rviz`

---

### Phase 6: Integration and Testing

#### Step 11: System Integration Test
- [ ] Connect Mid360 sensor via Ethernet
- [ ] Configure network settings (static IP recommended)
- [ ] Launch Livox driver and verify data publication
- [ ] Check topic output with `rostopic echo` and `rostopic hz`
- [ ] Verify IMU data is being published

**Network setup for Mid360:**
- Set computer IP to `192.168.1.50` (or similar)
- Mid360 default IP: `192.168.1.1XX`

#### Step 12: FAST-LIO2 Functional Test
- [ ] Launch FAST-LIO2 with Mid360
- [ ] Monitor console for initialization messages
- [ ] Check for errors or warnings
- [ ] Verify map is being built (check `/cloud_registered` topic)
- [ ] Observe odometry output
- [ ] Test in static and moving scenarios

#### Step 13: RVIZ Visualization Test
- [ ] Launch complete system with RVIZ
- [ ] Verify all displays show data correctly
- [ ] Test real-time performance (check frame rates)
- [ ] Validate point cloud registration quality
- [ ] Observe trajectory tracking
- [ ] Save RVIZ configuration

---

### Phase 7: Calibration and Tuning

#### Step 14: Sensor Calibration
- [ ] Verify/calibrate IMU to LiDAR extrinsics
- [ ] Test in different environments (indoor/outdoor)
- [ ] Check for drift in odometry
- [ ] Validate scale accuracy

#### Step 15: Parameter Tuning
- [ ] Tune mapping parameters:
  - `filter_size_surf` - Surface voxel filter size
  - `filter_size_map` - Map voxel filter size
  - `cube_side_length` - Local map size
- [ ] Adjust for specific use case:
  - Indoor: smaller filters, denser map
  - Outdoor: larger filters, faster processing
- [ ] Optimize for performance vs accuracy

#### Step 16: Performance Optimization
- [ ] Profile CPU usage
- [ ] Check memory consumption
- [ ] Optimize point cloud downsampling
- [ ] Adjust visualization settings for performance
- [ ] Document optimal parameter sets

---

### Phase 8: Documentation and Testing

#### Step 17: Create Usage Documentation
- [ ] Write README with setup instructions
- [ ] Document launch commands
- [ ] Add troubleshooting section
- [ ] Include parameter descriptions
- [ ] Add example outputs/screenshots

#### Step 18: Comprehensive Testing
- [ ] Test in various environments:
  - Indoor spaces
  - Outdoor areas
  - Mixed environments
- [ ] Test motion scenarios:
  - Static mapping
  - Slow movement
  - Fast movement
  - Rotation
- [ ] Record rosbags for testing
- [ ] Validate mapping quality

#### Step 19: Create Example Datasets
- [ ] Record sample rosbag files
- [ ] Include different scenarios
- [ ] Provide ground truth if available
- [ ] Add playback instructions

---

## Expected Directory Structure

```
Mid360-nav/
├── src/
│   ├── livox_ros_driver2/
│   ├── FAST_LIO/
│   └── ikd-Tree/
├── launch/
│   ├── livox_mid360.launch
│   ├── fast_lio_mid360.launch
│   ├── full_system.launch
│   └── visualization.launch
├── config/
│   ├── mid360_driver_config.json
│   ├── mid360_fastlio.yaml
│   └── rviz_config.rviz
├── scripts/
│   ├── setup_network.sh
│   └── record_bag.sh
├── docs/
│   ├── calibration.md
│   ├── troubleshooting.md
│   └── parameter_tuning.md
└── README.md
```

---

## Key Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Raw LiDAR point cloud |
| `/livox/imu` | `sensor_msgs/Imu` | IMU data from Mid360 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Registered point cloud map |
| `/Odometry` | `nav_msgs/Odometry` | Estimated pose |
| `/path` | `nav_msgs/Path` | Trajectory path |

---

## Resources and References

- **FAST-LIO2**: https://github.com/hku-mars/FAST_LIO
- **Livox SDK2**: https://github.com/Livox-SDK/Livox-SDK2
- **Livox ROS Driver2**: https://github.com/Livox-SDK/livox_ros_driver2
- **Mid360 Documentation**: Livox official website
- **Paper**: "FAST-LIO2: Fast Direct LiDAR-inertial Odometry"

---

## Success Criteria

- ✅ Mid360 publishes data at expected rate (10Hz typically)
- ✅ FAST-LIO2 processes in real-time (<100ms latency)
- ✅ Map builds correctly with minimal drift
- ✅ RVIZ displays all elements smoothly
- ✅ System runs continuously without crashes
- ✅ Odometry accuracy verified in test scenarios

---

## Next Steps After Implementation

1. **Add navigation capabilities** (move_base, path planning)
2. **Implement loop closure detection** for large-scale mapping
3. **Add map saving/loading functionality**
4. **Integrate with robot base** (if applicable)
5. **Develop autonomous navigation behaviors**

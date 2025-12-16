# GitHub Copilot Instructions for Mid360 Navigation Project

## Project Overview
This is a ROS-based SLAM and navigation system using FAST-LIO2 with the Livox Mid360 LiDAR sensor.

## Technology Stack & Design Decisions

### Core Technologies
- **ROS Distribution**: ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
  - Reason: Best compatibility with FAST-LIO2 and Livox drivers
- **SLAM Algorithm**: FAST-LIO2
  - Reason: State-of-the-art performance with Livox sensors, real-time capability
- **Sensor**: Livox Mid360 LiDAR with built-in IMU
- **Build System**: catkin (catkin_make or catkin build)
- **Visualization**: RVIZ

### Key Dependencies
- Livox SDK2 (for Mid360 support)
- livox_ros_driver2 (ROS interface for Mid360)
- ikd-Tree (incremental k-d tree for FAST-LIO2)
- PCL (Point Cloud Library) >= 1.8
- Eigen3
- Ceres Solver (optional, for optimization)

## Coding Standards & Conventions

### ROS Conventions
- Follow standard ROS naming conventions:
  - Packages: lowercase with underscores (e.g., `mid360_navigation`)
  - Nodes: lowercase with underscores
  - Topics: lowercase with forward slashes (e.g., `/livox/lidar`)
  - Parameters: lowercase with underscores
- Use proper ROS message types (sensor_msgs, nav_msgs, etc.)
- Always include proper dependencies in package.xml and CMakeLists.txt

### File Organization
```
project_root/
├── src/               # ROS packages (submodules/cloned repos)
├── launch/            # Launch files for system startup
├── config/            # Configuration files (YAML, JSON, RVIZ)
├── scripts/           # Utility scripts (bash, python)
├── docs/              # Documentation
└── data/              # Sample data, rosbags (not committed)
```

### Configuration Files
- **YAML** for ROS parameters
- **JSON** for Livox driver configuration
- **RVIZ** config files for visualization presets
- Keep configs modular (separate files for driver, FAST-LIO, RVIZ)

### Launch Files
- Create modular launch files that can be included/composed
- Use clear naming: `<component>_<sensor>.launch`
- Include comments explaining parameters
- Provide default values for all parameters
- Example structure:
  ```xml
  <!-- Driver only -->
  livox_mid360.launch
  
  <!-- FAST-LIO with driver -->
  fast_lio_mid360.launch
  
  <!-- Full system with visualization -->
  full_system.launch
  ```

## Code Style Guidelines

### Python Scripts
- Follow PEP 8
- Use ROS Python conventions (rospy)
- Include proper shebang: `#!/usr/bin/env python3`
- Make scripts executable: `chmod +x script.py`

### C++ Code
- Follow ROS C++ style guide
- Use modern C++ (C++14 or later)
- Proper header guards
- Initialize variables
- Use smart pointers where appropriate

### Launch File XML
- Proper indentation (2 spaces)
- Comment all non-obvious parameters
- Group related nodes together
- Use `<group>` tags for organization

### YAML Configuration
- Use consistent indentation (2 spaces)
- Comment all parameters with units and ranges
- Organize by functional sections
- Example:
  ```yaml
  # LiDAR Settings
  common:
      lid_topic: "/livox/lidar"    # CustomMsg format
      imu_topic: "/livox/imu"      # 200Hz IMU data
      time_sync_en: false          # No external time sync
  ```

## Naming Conventions

### Topics
- `/livox/lidar` - Raw LiDAR data from Mid360
- `/livox/imu` - IMU data from Mid360
- `/cloud_registered` - Registered point cloud map from FAST-LIO2
- `/Odometry` - Pose estimation from FAST-LIO2
- `/path` - Trajectory path

### Frames (TF)
- `world` or `map` - Global reference frame
- `camera_init` - FAST-LIO2 initialization frame
- `body` - Robot/sensor body frame
- `lidar` - LiDAR frame

### Parameters
Use descriptive names with context:
- `mid360/ip_address`
- `fast_lio/filter_size_surf`
- `mapping/max_range`

## Git Workflow

### Commit Messages
Follow conventional commits:
- `feat:` New feature
- `fix:` Bug fix
- `config:` Configuration changes
- `docs:` Documentation
- `refactor:` Code refactoring
- `test:` Testing

Example: `feat: add RVIZ configuration for Mid360 mapping`

### Branches
- `main` - Stable, tested code
- `develop` - Active development
- `feature/<name>` - New features
- `fix/<name>` - Bug fixes

### What NOT to Commit
- Build artifacts (build/, devel/, install/)
- Rosbag files (*.bag) unless small example files
- Large point cloud files (*.pcd, *.ply)
- IDE-specific files (.vscode/, .idea/)
- Compiled binaries

## Parameter Tuning Guidelines

### FAST-LIO2 Parameters

**For Indoor Environments:**
- Smaller voxel sizes (0.3-0.5m)
- Smaller local map size
- Lower minimum distance threshold

**For Outdoor Environments:**
- Larger voxel sizes (0.5-1.0m)
- Larger local map size
- Higher minimum distance threshold

**Common Parameters to Tune:**
- `filter_size_surf`: Surface voxel downsampling (0.3-0.5m typical)
- `filter_size_map`: Map voxel downsampling (0.3-0.5m typical)
- `cube_side_length`: Local map size (100-200m typical)
- `blind`: Minimum detection distance (0.5-2.0m)

## Testing & Validation

### Before Committing
- [ ] Code compiles without warnings
- [ ] Launch files tested and working
- [ ] Parameters documented
- [ ] No hardcoded paths (use rospack find, etc.)
- [ ] Proper error handling

### Testing Checklist
- Test with live sensor
- Test with recorded rosbag
- Verify RVIZ visualization
- Check topic publication rates
- Monitor CPU/memory usage
- Validate mapping quality

## Documentation Requirements

### README Updates
Update README.md when:
- Adding new dependencies
- Changing launch procedure
- Adding new features
- Modifying configuration

### Inline Documentation
- Comment complex algorithms
- Explain parameter choices
- Document coordinate frame conventions
- Note any calibration requirements

## Common Pitfalls to Avoid

### Livox Mid360 Specific
- ❌ Don't use Livox SDK1 (use SDK2 for Mid360)
- ❌ Don't use old livox_ros_driver (use livox_ros_driver2)
- ✅ Ensure correct network configuration (static IP recommended)
- ✅ Use `lidar_type: 1` in FAST-LIO2 config for Livox

### FAST-LIO2 Specific
- ❌ Don't use wrong point cloud type (use CustomMsg from Livox)
- ❌ Don't set filter sizes too small (causes performance issues)
- ✅ Verify IMU to LiDAR extrinsics
- ✅ Check time synchronization between sensors

### ROS Specific
- ❌ Don't hardcode absolute paths
- ❌ Don't assume package locations
- ✅ Use `$(find package_name)` in launch files
- ✅ Source workspace in scripts

## Performance Targets

- **Point Cloud Rate**: 10Hz (typical for Mid360)
- **FAST-LIO Processing**: Real-time (<100ms per frame)
- **CPU Usage**: <80% on target platform
- **Memory**: Monitor for leaks in long-running tests

## Future Considerations

Document here as the project evolves:
- Navigation stack integration
- Multi-sensor fusion
- Loop closure detection
- Map persistence (save/load)
- Path planning algorithms
- Obstacle detection

## Questions to Ask Before Implementation

1. Is this change compatible with ROS Noetic?
2. Does this require new dependencies? (Update installation docs)
3. Will this work with the Mid360 message format?
4. Is the coordinate frame convention consistent?
5. Are parameters configurable (not hardcoded)?
6. Does this need testing with both live sensor and rosbag?

## Helpful Commands Reference

```bash
# Build workspace
catkin_make
# or
catkin build

# Source workspace
source devel/setup.bash

# Check topics
rostopic list
rostopic echo /livox/lidar -n 1
rostopic hz /livox/lidar

# Check TF tree
rosrun tf view_frames
rosrun tf tf_echo source_frame target_frame

# Record data
rosbag record -a  # Record all topics
rosbag record /livox/lidar /livox/imu  # Specific topics

# Play back data
rosbag play <bagfile.bag>
```

---

*Last Updated: December 16, 2025*
*Update this file as design decisions are made throughout the project*

# Installation Guide for Mid360 Navigation System

## Prerequisites

- Ubuntu 20.04 (Focal Fossa) - Recommended
- Stable internet connection
- Sudo privileges

## Step-by-Step Installation

### Step 1: Install ROS Noetic

We've provided a convenient installation script:

```bash
cd ~/Documents/Mid360\ nav/scripts
chmod +x install_ros_noetic.sh
sudo bash install_ros_noetic.sh
```

After installation, source ROS:
```bash
source ~/.bashrc
# Or
source /opt/ros/noetic/setup.bash
```

Verify installation:
```bash
rosversion -d
# Should output: noetic
```

### Step 2: Create Catkin Workspace

Run the workspace setup script:

```bash
cd ~/Documents/Mid360\ nav/scripts
chmod +x setup_workspace.sh
bash setup_workspace.sh
```

This script will:
- Create `~/catkin_ws` directory structure
- Initialize the workspace with `catkin_make`
- Install essential ROS dependencies (PCL, Eigen, tf, etc.)
- Add workspace sourcing to your `.bashrc`

### Step 3: Verify Workspace Setup

```bash
source ~/.bashrc
echo $ROS_PACKAGE_PATH
# Should include: /home/your_username/catkin_ws/src
```

Test catkin:
```bash
cd ~/catkin_ws
catkin_make
# Should complete without errors
```

## Manual Installation (Alternative)

If you prefer to install manually:

### Install ROS Noetic

```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS
sudo apt update
sudo apt install ros-noetic-desktop-full

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### Create Workspace

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# Source workspace
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install dependencies
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-message-filters \
    ros-noetic-image-transport \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    libpcl-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev
```

## Troubleshooting

### ROS Installation Issues

**Issue**: `Unable to locate package ros-noetic-desktop-full`
- Solution: Ensure you're running Ubuntu 20.04 and have run `sudo apt update`

**Issue**: `rosdep init` fails with "already initialized"
- Solution: Skip this step, it's already initialized

### Workspace Issues

**Issue**: `catkin_make` command not found
- Solution: Source ROS first: `source /opt/ros/noetic/setup.bash`

**Issue**: `$ROS_PACKAGE_PATH` is empty
- Solution: Source your workspace: `source ~/catkin_ws/devel/setup.bash`

### Dependency Issues

**Issue**: PCL or Eigen not found during compilation
- Solution: 
  ```bash
  sudo apt-get install libpcl-dev libeigen3-dev
  ```

## Next Steps

After successful installation, proceed to:
1. Install Livox SDK2 and ROS driver
2. Install FAST-LIO2
3. Configure for Mid360 sensor

See [IMPLEMENTATION_PLAN.md](../IMPLEMENTATION_PLAN.md) for detailed steps.

## Verification Checklist

- [ ] ROS Noetic installed (`rosversion -d` works)
- [ ] Catkin workspace created at `~/catkin_ws`
- [ ] Workspace builds successfully (`catkin_make`)
- [ ] ROS environment sourced (check `$ROS_PACKAGE_PATH`)
- [ ] Essential dependencies installed (PCL, Eigen, tf, etc.)
- [ ] Can run basic ROS commands (`roscore`, `rostopic list`)

## System Requirements

**Minimum:**
- CPU: Dual-core processor
- RAM: 4GB
- Storage: 20GB free space

**Recommended:**
- CPU: Quad-core processor or better
- RAM: 8GB or more
- Storage: 50GB free space
- GPU: Optional but helpful for visualization

---

*For issues or questions, refer to the troubleshooting section or check ROS documentation at wiki.ros.org*

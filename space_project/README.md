# Space Project - ROS2 Navigation with A* Algorithm

This ROS2 package implements multi-robot navigation using the A* pathfinding algorithm. The robots navigate through an occupancy grid map to reach their target destinations.

## Prerequisites

- ROS2 Jazzy (or compatible version)
- Python 3.12+
- Ubuntu/Linux environment
- Required ROS2 packages:
  - `rclpy`
  - `nav_msgs`
  - `geometry_msgs`

## Project Structure

```
space_project/
├── space_project/
│   ├── __init__.py
│   ├── astar_navigation_node.py    # A* navigation controller
│   └── battery_manager.py          # Battery management node
├── launch/
│   └── multi_robots.launch.py      # Multi-robot launch file
├── package.xml
├── setup.py
└── README.md
```

## Building the Workspace

### Step 1: Navigate to your ROS2 workspace

```bash
cd ~/ros2_ws
```

### Step 2: Build all required packages

Build all packages in the workspace (recommended for first-time setup):

```bash
colcon build --symlink-install
```

Or build specific packages individually:

```bash
# Build space_project
colcon build --packages-select space_project --symlink-install

# Build ROS-TCP-Endpoint (required for Unity communication)
colcon build --packages-select ros_tcp_endpoint --symlink-install
```

The `--symlink-install` flag creates symbolic links instead of copying files, making development faster.

### Step 3: Source the workspace

After building, source the setup file to add the package to your environment:

```bash
source install/setup.bash
```

**Important:** You need to source this file in every new terminal session, or add it to your `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Launching the Application

### Launch Multi-Robot Navigation

Start the multi-robot navigation system with the default configuration:

```bash
ros2 launch space_project multi_robots.launch.py
```

This will start two robot controllers:
- **robot_0**: Navigates from (49.0, 49.0) to (40.0, 40.0)
- **robot_1**: Navigates from (20.0, 20.0) to (10.0, 10.0)

### Launch with Custom Parameters

You can launch individual nodes with custom source and destination coordinates:

```bash
ros2 run space_project astar_navigation_node --ros-args \
  -r __ns:=/robot_custom \
  -p src_x:=30.0 \
  -p src_y:=30.0 \
  -p dest_x:=10.0 \
  -p dest_y:=10.0
```

## Unity Integration

### Starting the ROS-TCP-Endpoint Server

To enable communication between ROS2 and Unity, you need to start the TCP endpoint server. This must be running before launching Unity.

#### Step 1: Get your system IP address

```bash
hostname -I | awk '{print $1}'
```

Your current IP is: **172.18.216.162**

#### Step 2: Start the TCP endpoint server

Open a **new terminal** (keep the navigation nodes running in the first terminal) and run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.18.216.162 -p ROS_TCP_PORT:=10000
```

**Important parameters:**
- `ROS_IP`: Your system's IP address (use the output from Step 1)
- `ROS_TCP_PORT`: The port Unity will connect to (default: 10000)

#### Step 3: Configure Unity

In your Unity project, configure the ROS connection settings to:
- **IP Address:** 172.18.216.162
- **Port:** 10000

### Complete Launch Sequence

For a full system startup with Unity integration, use **two separate terminals**:

**Terminal 1 - Navigation Nodes:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch space_project multi_robots.launch.py
```

**Terminal 2 - TCP Endpoint:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.18.216.162 -p ROS_TCP_PORT:=10000
```

Then start your Unity application.

## Expected Behavior

When launched successfully, you should see output similar to:

```
[INFO] [launch]: All log files can be found below ~/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [astar_navigation_node-1]: process started with pid [...]
[INFO] [astar_navigation_node-2]: process started with pid [...]
[astar_navigation_node-1] [INFO] [timestamp] [robot_0.controller]: Robot starting at src=(49.0, 49.0) going to dest=(40.0, 40.0)
[astar_navigation_node-2] [INFO] [timestamp] [robot_1.controller]: Robot starting at src=(20.0, 20.0) going to dest=(10.0, 10.0)
```

## ROS2 Topics

### Subscribed Topics

- `/map` (nav_msgs/OccupancyGrid): The occupancy grid map for navigation
- `/target` (geometry_msgs/PoseArray): Target positions for path planning

### Published Topics

- `/astar_path` (nav_msgs/Path): The computed A* path in Unity coordinates

## How It Works

1. **Map Reception**: Each robot controller subscribes to the `/map` topic to receive the occupancy grid
2. **Coordinate Conversion**: Unity coordinates (-50 to +50) are converted to grid indices
3. **A* Algorithm**: When a target is received, the A* algorithm computes the optimal path avoiding obstacles
4. **Path Publishing**: The computed path is published as a sequence of waypoints in Unity coordinates

## Troubleshooting

### Issue: "Package 'space_project' not found"

**Solution:**
```bash
# Rebuild the package
colcon build --packages-select space_project --symlink-install

# Source in a NEW terminal or re-source in current terminal
source install/setup.bash
```

### Issue: "Package 'ros_tcp_endpoint' not found"

**Solution:**
```bash
# Rebuild the ROS-TCP-Endpoint package
colcon build --packages-select ros_tcp_endpoint --symlink-install

# Source in a NEW terminal or re-source in current terminal
source install/setup.bash
```

### Issue: Unity can't connect to ROS

**Possible causes:**
1. TCP endpoint server not running
2. Wrong IP address or port
3. Firewall blocking the connection

**Solution:**
```bash
# Verify TCP endpoint is running
ros2 node list | grep tcp

# Check your current IP address
hostname -I

# Make sure the IP in Unity matches your ROS_IP parameter
# Make sure firewall allows port 10000
```

### Issue: Nodes start but don't do anything

**Cause:** The nodes are waiting for map and target data.

**Solution:** Make sure your Unity environment or map publisher is running and publishing to `/map` and `/target` topics.

Check if topics are available:
```bash
ros2 topic list
ros2 topic echo /map
```

### Issue: Setup file not found

**Solution:** Make sure you're running the source command from the workspace root:
```bash
cd ~/ros2_ws
source install/setup.bash
```

### Issue: Changes to Python code not reflected

**Solution:**
If you used `--symlink-install`, Python changes should be immediate. If not, rebuild:
```bash
colcon build --packages-select space_project
```

For changes to `setup.py`, always rebuild:
```bash
colcon build --packages-select space_project --symlink-install
```

## Development Workflow

### Making Changes

1. Edit Python files in `space_project/space_project/`
2. If you used `--symlink-install`, changes are immediate
3. If you modified `setup.py` or added new files:
   ```bash
   colcon build --packages-select space_project --symlink-install
   source install/setup.bash
   ```

### Testing Individual Nodes

Test the A* navigation node:
```bash
ros2 run space_project astar_navigation_node
```

Test the battery manager:
```bash
ros2 run space_project battery_manager
```

### Viewing Node Info

Check running nodes:
```bash
ros2 node list
```

Get node information:
```bash
ros2 node info /robot_0/controller
```

### Checking Logs

View logs in real-time:
```bash
ros2 run rqt_console rqt_console
```

Or check log files:
```bash
cd ~/.ros/log/latest/
```

## Quick Reference

```bash
# ===== BUILD =====
# Build all packages
colcon build --symlink-install

# Build specific packages
colcon build --packages-select space_project --symlink-install
colcon build --packages-select ros_tcp_endpoint --symlink-install

# ===== SOURCE =====
source install/setup.bash

# ===== LAUNCH =====
# Navigation nodes
ros2 launch space_project multi_robots.launch.py

# Unity TCP endpoint (in separate terminal)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.18.216.162 -p ROS_TCP_PORT:=10000

# Get your IP address
hostname -I | awk '{print $1}'

# ===== MONITORING =====
# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /astar_path

# List nodes
ros2 node list

# Node info
ros2 node info /robot_0/controller

# Check if TCP endpoint is running
ros2 node list | grep tcp
```
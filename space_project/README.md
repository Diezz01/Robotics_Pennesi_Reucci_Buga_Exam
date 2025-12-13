# Space Project - Battery-Aware Robot Navigation

This ROS2 package implements battery-aware multi-robot navigation using the A* pathfinding algorithm. Robots navigate through an occupancy grid map while monitoring battery levels and autonomously returning to charging stations when needed.

## Features

- ✅ **A* Pathfinding**: Optimal path planning with obstacle avoidance
- ✅ **Multi-Robot Collision Avoidance**: Real-time collision detection with predictive algorithm
- ✅ **Path Approval Service**: Centralized path coordination to prevent conflicts
- ✅ **Battery Management**: Real-time battery monitoring and health metrics
- ✅ **Autonomous Charging**: Robots automatically navigate to charging stations when battery is low
- ✅ **Priority-Based Coordination**: Distance-to-goal prioritization for conflict resolution
- ✅ **Deadlock Detection**: Automatic recovery from deadlock situations
- ✅ **Unity Integration**: Seamless communication with Unity simulation via ROS-TCP-Connector
- ✅ **Multi-Robot Support**: Configurable for up to 6 robots with independent coordination

## Prerequisites

- **ROS2 Jazzy** (or compatible version)
- **Python 3.12+**
- **Ubuntu/Linux environment**
- **Unity 2022.3+** (for simulation)
- Required ROS2 packages:
  - `rclpy`
  - `nav_msgs`
  - `geometry_msgs`
  - `std_msgs`
  - `ros_tcp_endpoint`

## Project Structure

```
space_project/
├── space_project/
│   ├── __init__.py
│   ├── astar_navigation_node.py        # A* navigation controller with path approval
│   ├── collision_coordinator_node.py   # Multi-robot collision avoidance & coordination
│   └── battery_manager.py              # Battery monitoring and health metrics
├── srv/
│   └── PathApproval.srv                # Path approval service definition
├── launch/
│   ├── multi_robots.launch.py          # Multi-robot launch file
│   └── battery_system.launch.py        # Complete battery management system
├── package.xml
├── setup.py
└── README.md
```

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Unity Simulation                          │
│  ┌────────────────┐  ┌──────────────┐  ┌─────────────────┐ │
│  │ExplorerController│  │BatterySimulator│  │ Charging Zone  │ │
│  │ (Mission Logic) │  │  (Physics)    │  │   (Trigger)    │ │
│  └────────────────┘  └──────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
            │                    │
            │ /target            │ /battery_state
            │ /astar_path        │ /charging_status
            ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 System (Python)                       │
│  ┌──────────────────┐         ┌────────────────────────┐   │
│  │ astar_navigation │◄────────┤   battery_manager      │   │
│  │  Path Planning   │         │  Health Monitoring     │   │
│  └──────────────────┘         └────────────────────────┘   │
│           │                                                  │
│           │ PathApproval Service                            │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         collision_coordinator_node                    │  │
│  │  Multi-Robot Collision Avoidance & Path Approval     │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         ROS-TCP-Endpoint (Unity Bridge)              │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Collision Coordinator Node

**Purpose:** Centralized multi-robot collision avoidance and path coordination

**Features:**
- **Real-time collision detection** (10 Hz) with velocity-based prediction
- **Path approval service** for pre-movement coordination
- **Priority-based conflict resolution** using distance-to-goal
- **Deadlock detection** with automatic recovery (8-second timeout)
- **Hysteresis control** to prevent state oscillation (0.5s cooldown)

**Launch:**
```bash
ros2 run space_project collision_coordinator_node --ros-args -p num_robots:=6
```

**Parameters:**
- `num_robots` (int, default: 6): Number of robots in system
- `safe_distance` (float, default: 4.0): Minimum separation (meters)
- `path_buffer` (float, default: 3.0): Planning buffer (meters)
- `collision_warning_distance` (float, default: 6.0): Early warning (meters)

**Subscribed Topics (per robot):**
- `/tb3_{i}/pose` (PoseStamped): Robot position for collision detection

**Published Topics (per robot):**
- `/tb3_{i}/collision_detected` (Bool): Emergency stop signal

**Services:**
- `/request_path_approval` (PathApproval): Approve/reject path requests before planning

**Algorithm:**
1. **Predictive Detection**: Uses velocity-based prediction (1s horizon) to prevent collisions
2. **Priority Resolution**: Lower priority robots stop (priority = distance_to_goal + robot_id × 0.01)
3. **Deadlock Recovery**: Clears collision flags after 8s timeout
4. **Hysteresis**: 0.5s cooldown between state changes to prevent oscillation

**Code:** `space_project/collision_coordinator_node.py`

### Battery Management Flow

1. **Battery Simulation** (Unity): BatterySimulator drains battery based on distance traveled
2. **State Publishing** (Unity → ROS2): Current battery level published to `/tb3_0/battery_state`
3. **Health Monitoring** (ROS2): battery_manager tracks drain rate and publishes metrics
4. **Mission Planning** (Unity): ExplorerController checks battery before each mission
5. **Autonomous Charging**: If battery insufficient, navigate to charging station first
6. **Resume Mission**: After charging, automatically resume pending missions

## Installation

### Step 1: Navigate to your ROS2 workspace

```bash
cd ~/ros2_ws
```

### Step 2: Build the package

**First time setup (build all packages):**
```bash
colcon build --symlink-install
```

**Or build specific package:**
```bash
colcon build --packages-select space_project --symlink-install
```

The `--symlink-install` flag creates symbolic links for faster development.

### Step 3: Source the workspace

```bash
source install/setup.bash
```

**Pro tip:** Add to `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Launching the System

### Option 1: Launch Complete Battery Management System (Recommended)

This launches all necessary nodes in one command:

```bash
ros2 launch space_project battery_system.launch.py
```

**What this starts:**
- ✅ ROS-TCP-Endpoint (Unity communication on port 10000)
- ✅ A* Navigation Node (path planning)
- ✅ Battery Manager (monitoring and health metrics)

**With custom parameters:**
```bash
ros2 launch space_project battery_system.launch.py \
  robot_name:=tb3_0 \
  low_battery_threshold:=30.0 \
  critical_battery_threshold:=15.0 \
  charged_threshold:=95.0 \
  publish_metrics_interval:=5.0
```

### Option 2: Launch Nodes Separately (For Debugging)

**Terminal 1 - ROS-TCP-Endpoint:**
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Terminal 2 - A* Navigation:**
```bash
ros2 run space_project astar_navigation_node
```

**Terminal 3 - Battery Manager:**
```bash
ros2 run space_project battery_manager --ros-args \
  -p robot_name:=tb3_0 \
  -p low_battery_threshold:=30.0 \
  -p critical_battery_threshold:=15.0 \
  -p charged_threshold:=95.0
```

### Option 3: Multi-Robot Navigation (Legacy)

```bash
ros2 launch space_project multi_robots.launch.py
```

## Unity Setup

### 1. Configure ROSConnection GameObject

In Unity Scene Hierarchy, find **ROSConnectionPrefab**:

**ROS Connection Settings:**
- **ROS IP Address**: `127.0.0.1` (if same machine) or your ROS machine IP
- **ROS Port**: `10000`
- ✅ **Connect on Startup**: Enabled

### 2. Configure BatterySimulator Component

On your robot GameObject:

**Battery Settings:**
- Max Charge: `100`
- Current Charge: `100`
- Discharge Per Meter: `0.5`
- Discharge Per Second Idle: `0.01`
- Charge Per Second: `5`

**ROS Topics:**
- Battery Topic: `/tb3_0/battery_state`
- Charging State Topic: `/tb3_0/charging_status`

### 3. Configure ExplorerController Component

**Battery Management:**
- Charging Station Position: `(12, 0, -38)` (match your scene)
- Battery Discharge Per Meter: `0.5` (for cost estimation)
- Low Battery Threshold: `30`
- Charged Threshold: `95`
- Safety Cost Multiplier: `1.2`
- Battery Topic Name: `/tb3_0/battery_state`

### 4. Create Charging Zone

1. Create GameObject at charging station location (e.g., `12, 0, -38`)
2. Add **Collider** component (Box or Sphere)
3. ✅ Check **"Is Trigger"**
4. Set Tag to **"ChargingZone"**
5. Ensure robot has **Rigidbody** component

## Complete Startup Sequence

### Step 1: Start ROS2 System

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch space_project battery_system.launch.py
```

**Expected output:**
```
============================================================
Battery Management System Launched
============================================================
Robot Name: tb3_0
Low Battery Threshold: 30.0%
Critical Battery Threshold: 15.0%
Charged Threshold: 95.0%
Metrics Interval: 5.0s
============================================================

Monitor battery with:
  ros2 topic echo /tb3_0/battery_state
  ros2 topic echo /tb3_0/battery_health
```

### Step 2: Start Unity

1. Open Unity project
2. Press **Play**
3. Check Unity Console for:
   ```
   ✅ BatterySimulator: ROS connected. Registered publishers...
   ✅ ExplorerController: Battery level received: 100.0%
   ```

### Step 3: Monitor System (Optional)

**Terminal 2 - Monitor Battery State:**
```bash
ros2 topic echo /tb3_0/battery_state
```

**Terminal 3 - Monitor Health Metrics:**
```bash
ros2 topic echo /tb3_0/battery_health
```

**Terminal 4 - Monitor Alerts:**
```bash
ros2 topic echo /tb3_0/battery_alert
```

## ROS2 Services

### PathApproval Service

**Service:** `/request_path_approval`
**Type:** `space_project/PathApproval`
**Purpose:** Multi-robot path coordination - request approval before planning path

#### Service Definition

**Request:**
```
int32 robot_id              # Requesting robot index (0-5)
float64 start_x             # Current X position
float64 start_y             # Current Y position (Unity Z)
float64 goal_x              # Target X position
float64 goal_y              # Target Y position (Unity Z)
```

**Response:**
```
bool approved               # true = approved, false = rejected
float32 wait_time           # Suggested wait time (0 if approved, 3.0 if rejected)
string reason               # Explanation (e.g., "Path clear" or "Wait for tb3_2")
```

#### Example Usage

**Test service call:**
```bash
ros2 service call /request_path_approval space_project/srv/PathApproval \
  "{robot_id: 0, start_x: 10.0, start_y: 20.0, goal_x: 30.0, goal_y: 40.0}"
```

**Expected Response (approved):**
```
approved: True
wait_time: 0.0
reason: "Path clear"
```

**Expected Response (rejected):**
```
approved: False
wait_time: 3.0
reason: "Wait for tb3_2 (5.2m away)"
```

#### Algorithm

The service evaluates requests using:
1. **Goal conflict check**: Compare goal with other robots' positions and goals
2. **Priority evaluation**: Calculate priority = distance_to_goal + (robot_id × 0.01)
3. **Decision**: Approve if no conflict OR requesting robot has higher priority (lower value)
4. **Retry**: If rejected, requester retries after wait_time (3.0 seconds)

## ROS2 Topics

### Per-Robot Topics

Each robot (tb3_0 through tb3_5) has its own dedicated topics:

#### Battery Topics

| Topic Pattern | Type | Direction | Purpose |
|---------------|------|-----------|---------|
| `/tb3_{i}/battery_state` | `std_msgs/Float32` | Unity → ROS2 | Current battery percentage (0-100) |
| `/tb3_{i}/charging_status` | `std_msgs/Bool` | Unity → ROS2 | Charging state (true/false) |
| `/tb3_{i}/battery_health` | `std_msgs/String` (JSON) | ROS2 | Battery health metrics |
| `/tb3_{i}/battery_alert` | `std_msgs/String` (JSON) | ROS2 | Battery alerts (LOW, CRITICAL) |

#### Navigation Topics

| Topic Pattern | Type | Direction | Purpose |
|---------------|------|-----------|---------|
| `/tb3_{i}/target` | `geometry_msgs/PoseArray` | Unity → ROS2 | Target positions [robot_pos, target_pos] |
| `/tb3_{i}/astar_path` | `nav_msgs/Path` | ROS2 → Unity | Computed A* path waypoints |

#### Collision Avoidance Topics

| Topic Pattern | Type | Direction | Purpose |
|---------------|------|-----------|---------|
| `/tb3_{i}/pose` | `geometry_msgs/PoseStamped` | Unity → ROS2 | Robot position for collision detection (10 Hz) |
| `/tb3_{i}/collision_detected` | `std_msgs/Bool` | ROS2 → Unity | Emergency stop signal (10 Hz) |

### Shared Topics

These topics are shared across all robots:

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | Unity → ROS2 | Occupancy grid for navigation |
| `/robots` | `geometry_msgs/PoseArray` | Unity → ROS2 | Initial robot spawn positions |
| `/targets` | `geometry_msgs/PoseArray` | Unity → ROS2 | Excavation point positions |

### Battery Health Message Format (JSON)

```json
{
  "timestamp": "2025-12-04T10:30:15",
  "battery_level": 45.2,
  "is_charging": false,
  "drain_rate_percent_per_second": 0.0234,
  "estimated_remaining_seconds": 1932,
  "status": "NORMAL"
}
```

**Status Values**: `CHARGING`, `CRITICAL`, `LOW`, `NORMAL`, `FULL`

## Monitoring and Debugging

### Check System Status

**List all nodes:**
```bash
ros2 node list
```
Expected:
```
/collision_coordinator_node
/astar_navigation
/battery_manager
/ros_tcp_endpoint
```

**Check battery topic connection:**
```bash
ros2 topic info /tb3_0/battery_state
```
Expected:
```
Publisher count: 1  # Unity (BatterySimulator)
Subscription count: 2  # ExplorerController + battery_manager
```

**View ROS graph:**
```bash
ros2 run rqt_graph rqt_graph
```

### Logging

**With verbose logging:**
```bash
ros2 launch space_project battery_system.launch.py --log-level DEBUG
```

**Save logs to file:**
```bash
ros2 launch space_project battery_system.launch.py 2>&1 | tee battery_$(date +%Y%m%d_%H%M%S).log
```

**Record topics for playback:**
```bash
ros2 bag record -a -o battery_session
# Playback later with:
ros2 bag play battery_session
```

**Check log files:**
```bash
cat ~/.ros/log/latest/battery_manager/stdout.log
```

## Expected Behavior

### Normal Operation

**Unity Console:**
```
✅ Starting mission to (5, 0, 10). Battery: 85.0% | Cost: 12.5%
🟢 Target reached: (5, 0, 10)
✅ Starting mission to (15, 0, 20). Battery: 72.5% | Cost: 18.0%
```

**ROS2 Terminal:**
```
[INFO] [battery_manager]: 📊 Battery: 85.0% | Status: NORMAL | Drain: 0.0234%/s
[INFO] [battery_manager]: 📊 Battery: 72.5% | Status: NORMAL | Drain: 0.0245%/s
```

### Low Battery Scenario

**Unity Console:**
```
⚠️ Insufficient battery for mission. Inserting charging station visit. Battery: 28.5% | Required: 45.0%
🟡 Arrived at charging station. Current battery: 28.5%. Waiting for charge...
🔌 Charging status: True | Battery: 28.5%
🟢 Battery charged to 95.0%. Resuming mission.
```

**ROS2 Terminal:**
```
[WARN] [battery_manager]: ⚠️ LOW BATTERY: 28.5%
[INFO] [battery_manager]: 🔌 Charging started at 28.5%
[INFO] [battery_manager]: ✅ Battery charged to 95.0%
```

## Troubleshooting

### Unity Side

**Issue: "No registered publisher on topic /tb3_0/battery_state"**

**Cause:** ROSConnection not configured or BatterySimulator not registering publishers.

**Solution:**
1. Check Unity Console for: `"BatterySimulator: ROS connected. Registered publishers..."`
2. Ensure ROSConnection GameObject exists in scene
3. Verify BatterySimulator component is attached and enabled

**Issue: Battery drains in Unity but not in ROS2**

**Solution:**
```bash
# Check if Unity is publishing
ros2 topic list | grep battery

# Check for publishers
ros2 topic info /tb3_0/battery_state

# Echo topic to see values
ros2 topic echo /tb3_0/battery_state
```

If no output:
1. Restart Unity
2. Check ROSConnection IP/Port settings
3. Ensure ROS-TCP-Endpoint is running

### ROS2 Side

**Issue: "Package 'space_project' not found"**

**Solution:**
```bash
colcon build --packages-select space_project --symlink-install
source install/setup.bash  # In NEW terminal or re-source
```

**Issue: "battery_manager not found"**

**Solution:**
```bash
# Check if entry point exists
grep battery_manager setup.py

# Rebuild
colcon build --packages-select space_project --symlink-install
source install/setup.bash
```

**Issue: Unity can't connect to ROS**

**Solution:**
```bash
# Check if TCP endpoint is running
ros2 node list | grep tcp

# Verify IP address
hostname -I

# Make sure firewall allows port 10000
sudo ufw allow 10000
```

**Issue: Nodes start but don't publish**

**Cause:** Waiting for Unity to send map and targets.

**Solution:**
1. Start Unity first
2. Check topics: `ros2 topic list`
3. Verify `/map` and `/target` topics exist

## Development

### Making Changes

**Python files:**
1. Edit files in `space_project/space_project/`
2. Changes are immediate (if using `--symlink-install`)
3. No rebuild needed for Python code changes

**Configuration files (setup.py, launch files):**
```bash
colcon build --packages-select space_project --symlink-install
source install/setup.bash
```

### Testing Individual Components

**Test A* navigation:**
```bash
ros2 run space_project astar_navigation_node
```

**Test battery manager:**
```bash
ros2 run space_project battery_manager --ros-args -p robot_name:=tb3_0
```

**Test with custom log level:**
```bash
ros2 run space_project battery_manager --ros-args -p robot_name:=tb3_0 --log-level DEBUG
```

## Parameters Reference

### battery_manager Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_name` | string | `tb3_0` | Robot identifier for topic namespacing |
| `low_battery_threshold` | float | `30.0` | Battery % to trigger low battery warning |
| `critical_battery_threshold` | float | `15.0` | Battery % to trigger critical alert |
| `charged_threshold` | float | `95.0` | Battery % to consider charging complete |
| `publish_metrics_interval` | float | `5.0` | Seconds between health metric publications |

## Quick Reference

```bash
# ===== BUILD =====
colcon build --packages-select space_project --symlink-install
source install/setup.bash

# ===== LAUNCH =====
# Complete system (recommended)
ros2 launch space_project battery_system.launch.py

# With custom parameters
ros2 launch space_project battery_system.launch.py \
  robot_name:=tb3_0 low_battery_threshold:=30.0

# Legacy multi-robot
ros2 launch space_project multi_robots.launch.py

# ===== MONITORING =====
# List all topics
ros2 topic list

# Monitor battery
ros2 topic echo /tb3_0/battery_state
ros2 topic echo /tb3_0/battery_health
ros2 topic echo /tb3_0/battery_alert

# List nodes
ros2 node list

# Node information
ros2 node info /battery_manager

# ===== DEBUGGING =====
# Launch with debug logging
ros2 launch space_project battery_system.launch.py --log-level DEBUG

# Check topic connections
ros2 topic info /tb3_0/battery_state

# View ROS graph
ros2 run rqt_graph rqt_graph

# Record session
ros2 bag record -a -o session_name

# ===== NETWORK =====
# Get your IP
hostname -I | awk '{print $1}'

# Check if endpoint is running
ros2 node list | grep tcp
```

## Contributing

When making changes:
1. Test with `--symlink-install` for faster iteration
2. Run with `--log-level DEBUG` to verify behavior
3. Use `ros2 bag record` to capture test sessions
4. Update this README with new features

## License

Apache-2.0

## Support

For issues related to:
- **ROS2 nodes**: Check logs in `~/.ros/log/latest/`
- **Unity integration**: Check Unity Console for ROS connection messages
- **Battery system**: Use `ros2 topic echo /tb3_0/battery_health` for diagnostics

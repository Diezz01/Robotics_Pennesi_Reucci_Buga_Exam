# Robotics Battery-Aware Navigation System

An integrated Unity-ROS2 system for autonomous robot navigation with intelligent battery management. Robots navigate through environments using A* pathfinding while monitoring battery levels and autonomously returning to charging stations when needed.

## Project Overview

This project implements a complete robotics simulation system combining:
- **Unity Simulation** - 3D environment with physics-based robot control and battery simulation
- **ROS2 Backend** - Path planning, battery monitoring, and health metrics
- **Battery Management** - Autonomous charging behavior with predictive mission planning

### Key Features

- ✅ **A* Pathfinding** - Optimal path planning with obstacle avoidance
- ✅ **Physics-Based Battery Simulation** - Realistic battery drain based on distance and idle time
- ✅ **Autonomous Charging** - Robots automatically navigate to charging stations
- ✅ **Predictive Mission Planning** - Checks battery sufficiency before starting missions
- ✅ **Real-Time Monitoring** - Battery health metrics, drain rate, and remaining time estimation
- ✅ **Multi-Robot Support** - Configurable for multiple independent robots
- ✅ **Unity-ROS2 Integration** - Seamless bi-directional communication

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         UNITY SIMULATION                             │
│                                                                       │
│  ┌──────────────────┐  ┌───────────────────┐  ┌──────────────────┐ │
│  │ ExplorerController │  │ BatterySimulator │  │  Charging Zone   │ │
│  │                   │  │                   │  │                  │ │
│  │ • Mission Queue   │  │ • Drain Physics   │  │ • Trigger Zone   │ │
│  │ • Cost Calculation│  │ • Charge Physics  │  │ • Auto-detect    │ │
│  │ • Decision Logic  │  │ • ROS Publishing  │  │                  │ │
│  └──────────────────┘  └───────────────────┘  └──────────────────┘ │
│           │                      │                                   │
│           │ Requests Paths       │ Publishes Battery                │
│           ▼                      ▼                                   │
└───────────────────────────────────────────────────────────────────────┘
            │                      │
            │ /target              │ /battery_state
            │ /astar_path          │ /charging_status
            ▼                      ▼
┌───────────────────────────────────────────────────────────────────────┐
│                         ROS2 SYSTEM                                   │
│                                                                       │
│  ┌────────────────────┐         ┌──────────────────────────────┐   │
│  │ A* Navigation Node │         │    Battery Manager Node      │   │
│  │                    │         │                              │   │
│  │ • Path Planning    │         │ • Health Monitoring          │   │
│  │ • Obstacle Avoid   │         │ • Drain Rate Calculation     │   │
│  │ • Grid Conversion  │         │ • Alert System               │   │
│  │                    │         │ • Metrics Publishing         │   │
│  └────────────────────┘         └──────────────────────────────┘   │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │              ROS-TCP-Endpoint (Unity Bridge)                 │   │
│  │              Handles Unity ↔ ROS2 Communication              │   │
│  └─────────────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
Robotics_Pennesi_Reucci_Buga_Exam/
├── Robotics_env_exam_Pennesi_Reucci/    # Unity Project
│   ├── Assets/
│   │   ├── Scenes/
│   │   │   └── SampleScene.unity         # Main simulation scene
│   │   ├── Scripts/
│   │   │   ├── ExplorerController.cs     # Mission planning & battery-aware navigation
│   │   │   ├── BatterySimulator.cs       # Physics-based battery simulation
│   │   │   ├── AStarController.cs        # Unity A* integration
│   │   │   ├── PathFollow.cs             # Path execution
│   │   │   ├── MapGenerator.cs           # Occupancy grid generation
│   │   │   └── OdomPublisher.cs          # Odometry publishing
│   │   └── Resources/
│   │       └── ROSConnectionPrefab.prefab # ROS connection configuration
│   └── ProjectSettings/
│
└── space_project/                         # ROS2 Package
    ├── space_project/
    │   ├── astar_navigation_node.py       # A* path planning
    │   └── battery_manager.py             # Battery monitoring & health metrics
    ├── launch/
    │   ├── battery_system.launch.py       # Complete system launch
    │   └── multi_robots.launch.py         # Multi-robot launch
    ├── package.xml
    ├── setup.py
    └── README.md                          # Detailed ROS2 documentation
```

## Components

### Unity Components

#### 1. ExplorerController
**Purpose**: Mission planning with battery-aware navigation

**Responsibilities**:
- Manages excavation point queue
- Calculates mission costs (target + return to charging)
- Decides when to insert charging station visits
- Publishes navigation targets to ROS2
- Receives and executes A* paths

**Key Parameters**:
- Charging Station Position: `(12, 0, -38)`
- Battery Discharge Per Meter: `0.5%`
- Low Battery Threshold: `30%`
- Charged Threshold: `95%`
- Safety Cost Multiplier: `1.2` (20% safety margin)

#### 2. BatterySimulator
**Purpose**: Physics-based battery simulation

**Responsibilities**:
- Simulates battery drain based on distance traveled
- Simulates idle battery consumption
- Detects charging zone (trigger-based)
- Simulates charging when in zone
- Publishes battery state to ROS2

**Key Parameters**:
- Max Charge: `100%`
- Discharge Per Meter: `0.5%`
- Discharge Per Second Idle: `0.01%`
- Charge Per Second: `5%`

#### 3. Charging Zone
**Setup**:
- GameObject with Collider (Is Trigger: ✓)
- Tag: `"ChargingZone"`
- Positioned at charging station location

**Behavior**:
- OnTriggerEnter: Start charging
- OnTriggerExit: Stop charging

### ROS2 Components

#### 1. A* Navigation Node
**Purpose**: Optimal path planning

**Responsibilities**:
- Receives target requests from Unity
- Plans collision-free paths using A* algorithm
- Converts between Unity and grid coordinates
- Publishes waypoint paths back to Unity

**Algorithm**: A* with Euclidean heuristic, 8-directional movement

#### 2. Battery Manager Node
**Purpose**: Battery health monitoring and metrics

**Responsibilities**:
- Monitors battery level from Unity
- Tracks battery drain rate
- Estimates remaining time
- Publishes health metrics (JSON)
- Sends alerts (LOW, CRITICAL)
- Does NOT control navigation (monitoring only)

**Thresholds**:
- Low Battery: `30%`
- Critical Battery: `15%`
- Charged: `95%`

## Communication Topics

### Battery Topics

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/tb3_0/battery_state` | `std_msgs/Float32` | Unity → ROS2 | ~60 Hz | Current battery percentage (0-100) |
| `/tb3_0/charging_status` | `std_msgs/Bool` | Unity → ROS2 | On change | Charging state (true/false) |
| `/tb3_0/battery_health` | `std_msgs/String` | ROS2 | 5s | Battery health metrics (JSON) |
| `/tb3_0/battery_alert` | `std_msgs/String` | ROS2 | On event | Battery alerts (JSON) |

### Navigation Topics

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Unity → ROS2 | Once | Static occupancy grid for pathfinding |
| `/target` | `geometry_msgs/PoseArray` | Unity → ROS2 | On demand | [robot_position, target_position] |
| `/astar_path` | `nav_msgs/Path` | ROS2 → Unity | On demand | Computed waypoint path |
| `/robot_pose` | `geometry_msgs/Pose` | Unity → ROS2 | ~30 Hz | Current robot pose |

### Message Formats

#### Battery Health (JSON String)
```json
{
  "timestamp": "2025-12-04T10:30:15.123456",
  "battery_level": 45.2,
  "is_charging": false,
  "drain_rate_percent_per_second": 0.0234,
  "estimated_remaining_seconds": 1932,
  "status": "NORMAL"
}
```

**Status Values**: `CHARGING`, `CRITICAL`, `LOW`, `NORMAL`, `FULL`

#### Battery Alert (JSON String)
```json
{
  "timestamp": "2025-12-04T10:30:15.123456",
  "level": "WARNING",
  "message": "Battery low: 28.5%",
  "battery_level": 28.5
}
```

**Alert Levels**: `INFO`, `WARNING`, `CRITICAL`

## Battery Management Flow

### Mission Planning Sequence

```
1. ExplorerController receives mission queue (excavation points)
   ↓
2. Before each mission:
   • Calculate distance to target
   • Calculate distance from target to charging station
   • Estimate total battery cost: (dist_to_target + dist_to_charging) × discharge_rate × safety_margin
   ↓
3. Decision:
   IF battery ≥ estimated_cost:
      ✅ Execute mission
   ELSE IF battery ≥ cost_to_charging_station:
      ⚠️ Insert charging station as priority target
   ELSE:
      🚨 CRITICAL: Robot stranded (insufficient battery to reach charging)
   ↓
4. If charging mission:
   • Navigate to charging station using A*
   • Enter charging zone (trigger detection)
   • BatterySimulator starts charging
   • Wait until battery ≥ charged_threshold (95%)
   • Resume mission queue
```

### Battery Monitoring Flow

```
Unity (BatterySimulator):
├─ Update(): Each frame
│  ├─ Calculate distance moved
│  ├─ Drain: battery -= (distance × discharge_per_meter)
│  ├─ Drain: battery -= (idle_rate × deltaTime)
│  ├─ Charge: battery += (charge_rate × deltaTime) [if in zone]
│  └─ Publish: /tb3_0/battery_state
│
ROS2 (battery_manager):
├─ Receives: /tb3_0/battery_state
├─ Updates: Battery history (last 100 samples)
├─ Calculates: Drain rate from history
├─ Estimates: Remaining time
├─ Publishes: Health metrics every 5 seconds
└─ Triggers: Alerts on threshold crossings
```

## Unity Setup

### Prerequisites
- Unity 2022.3 or newer
- ROS-TCP-Connector package
- Robot model with Rigidbody component

### Configuration Steps

**1. ROSConnection GameObject**
- ROS IP Address: `127.0.0.1` (or ROS machine IP)
- ROS Port: `10000`
- Connect on Startup: ✓

**2. Robot GameObject**
- Add `BatterySimulator` component
- Add `ExplorerController` component
- Ensure `Rigidbody` component exists

**3. Charging Zone GameObject**
- Position: Match `ExplorerController.chargingStationPosition`
- Add `Collider` component (Box or Sphere)
- Set `Is Trigger`: ✓
- Tag: `"ChargingZone"`

**4. Excavation Points**
- Create GameObjects at target locations
- Tag: `"ExcavationPoint"`

### Inspector Configuration

#### ExplorerController
```
Movement:
├─ Linear Speed: 4.0 m/s
├─ Angular Speed: 180 deg/s
└─ Reach Threshold: 0.01 m

Battery Management:
├─ Charging Station Position: (12, 0, -38)
├─ Battery Discharge Per Meter: 0.5
├─ Low Battery Threshold: 30
├─ Charged Threshold: 95
├─ Safety Cost Multiplier: 1.2
└─ Battery Topic Name: "/tb3_0/battery_state"

ROS Topics:
├─ Topic Name Target: "/target"
└─ Topic Name Path: "/astar_path"
```

#### BatterySimulator
```
Battery Settings:
├─ Max Charge: 100
├─ Current Charge: 100
├─ Discharge Per Meter: 0.5
├─ Discharge Per Second Idle: 0.01
└─ Charge Per Second: 5

ROS Topics:
├─ Battery Topic: "/tb3_0/battery_state"
└─ Charging State Topic: "/tb3_0/charging_status"
```

## ROS2 Setup

See `space_project/README.md` for detailed ROS2 setup, building, and launching instructions.

**Quick Start**:
```bash
cd ~/ros2_ws
colcon build --packages-select space_project --symlink-install
source install/setup.bash
ros2 launch space_project battery_system.launch.py
```

## Expected Behavior

### Scenario 1: Normal Mission Execution

**Unity Console**:
```
✅ Starting mission to (5, 0, 10). Battery: 85.0% | Cost: 12.5%
📍 Path received from ROS. Length: 15
🎯 Target reached: (5, 0, 10)
✅ Starting mission to (15, 0, 20). Battery: 72.5% | Cost: 18.0%
```

**Observation**:
- Robot moves smoothly along A* computed path
- Battery depletes as robot moves
- No charging needed (sufficient battery)

### Scenario 2: Low Battery - Autonomous Charging

**Unity Console**:
```
⚠️ Insufficient battery for mission. Battery: 28.5% | Required: 45.0%
🔋 Inserting charging station visit
📍 Navigating to charging station (12, 0, -38)
🎯 Arrived at charging station. Current battery: 26.2%
🔌 Charging status: True | Battery: 26.2%
⚡ Battery: 50.0%
⚡ Battery: 75.0%
⚡ Battery: 95.0%
✅ Battery charged to 95.0%. Resuming mission.
📍 Resuming mission queue
```

**Observation**:
- Robot detects insufficient battery before starting mission
- Automatically navigates to charging station
- Enters charging zone (trigger detection)
- Battery charges at 5% per second
- Resumes pending missions after charging

### Scenario 3: Critical Battery Error

**Unity Console**:
```
🚨 CRITICAL: Battery too low to reach charging station!
    Battery: 8.5% | Required: 12.0%
⛔ All operations stopped - robot stranded
```

**Observation**:
- Robot cannot reach charging station with remaining battery
- All operations halted to prevent further drain
- Manual intervention required

## Monitoring the System

### Unity Side

**Console Messages** (color-coded):
- 🟢 Green: Successful operations
- 🟡 Yellow: Warnings (low battery, charging needed)
- 🔴 Red: Critical errors
- 🔵 Cyan: Information (battery received, missions)
- 🟣 Magenta: Charging status changes

**Inspector View**:
- BatterySimulator: Real-time battery level display
- ExplorerController: Current target, mission status

### ROS2 Side

**Monitor Battery State**:
```bash
ros2 topic echo /tb3_0/battery_state
```

**Monitor Health Metrics**:
```bash
ros2 topic echo /tb3_0/battery_health
```

**Monitor Alerts**:
```bash
ros2 topic echo /tb3_0/battery_alert
```

**View System Graph**:
```bash
ros2 run rqt_graph rqt_graph
```

## Key Design Decisions

### Separation of Concerns

**BatterySimulator** (Unity):
- **Responsibility**: Physics simulation only
- Simulates battery drain and charging
- Publishes state to ROS2
- No decision-making logic

**ExplorerController** (Unity):
- **Responsibility**: Mission planning and execution
- Reads battery state from BatterySimulator (via ROS2)
- Makes navigation decisions
- Does NOT simulate battery

**battery_manager** (ROS2):
- **Responsibility**: Monitoring and metrics only
- Tracks battery health
- Calculates drain rates
- Publishes alerts
- Does NOT control navigation

### Why This Architecture?

1. **Modularity**: Each component has single responsibility
2. **Testability**: Components can be tested independently
3. **Reusability**: battery_manager can work with different robots
4. **Scalability**: Easy to add multiple robots
5. **Maintainability**: Changes to one component don't affect others

### Safety Features

**20% Safety Margin** (`safetyCostMultiplier = 1.2`):
- Prevents edge cases where estimates are too optimistic
- Accounts for path inefficiencies (not straight line)
- Provides buffer for unexpected drain

**Predictive Planning**:
- Checks battery BEFORE starting mission
- Calculates round-trip cost (target + return to charging)
- Prevents mid-mission battery depletion

**Critical Battery Detection**:
- Stops all operations if can't reach charging station
- Prevents robot from becoming stranded

## Multi-Robot Support

The system is designed for multiple robots:

**Unity Side**:
- Add robot name parameter to components
- Each robot has independent battery simulation
- Separate topic namespaces: `/tb3_0/...`, `/tb3_1/...`

**ROS2 Side**:
- Launch multiple `battery_manager` instances
- Each monitors different robot namespace
- Independent health tracking per robot

## Performance Considerations

**Unity**:
- Battery published at ~60 Hz (Update rate)
- Path following at ~60 Hz
- Minimal computational overhead

**ROS2**:
- Battery health metrics published every 5 seconds (configurable)
- A* computation on-demand only
- Efficient path planning with grid-based search

**Network**:
- All communication via ROS-TCP-Connector (port 10000)
- JSON messages for human-readable health metrics
- Binary messages for high-frequency data (battery state, poses)

## Troubleshooting

### Unity Not Publishing Battery

**Check**:
1. BatterySimulator component enabled?
2. ROSConnection GameObject in scene?
3. Unity Console shows: "BatterySimulator: ROS connected"?

### ROS2 Not Receiving Battery

**Check**:
```bash
ros2 topic list | grep battery  # Topics exist?
ros2 topic info /tb3_0/battery_state  # Publisher count = 1?
ros2 node list | grep tcp  # Endpoint running?
```

### Robot Not Charging

**Check**:
1. Charging Zone has "ChargingZone" tag?
2. Charging Zone has trigger collider?
3. Robot has Rigidbody component?
4. Unity Console shows: "Charging status: True"?

### Missions Not Starting

**Check**:
1. ExplorerController receives battery updates?
2. Excavation points tagged correctly?
3. A* navigation node running?
4. Map published to ROS2?

## Additional Resources

- **ROS2 Detailed Documentation**: `space_project/README.md`
- **Unity ROS-TCP-Connector**: https://github.com/Unity-Technologies/ROS-TCP-Connector
- **ROS2 Documentation**: https://docs.ros.org/

## License

Apache-2.0

## Contributors

- Kiril Buga
- Diego Pennesi
- Filippo Reucci

## Support

For detailed ROS2 launch instructions, debugging, and development workflow, see:
- `space_project/README.md` - Complete ROS2 documentation
- Unity Console - Real-time system status
- ROS2 logs: `~/.ros/log/latest/`

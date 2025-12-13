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
- ✅ **Multi-Robot Collision Avoidance** - Real-time collision detection with predictive algorithm
- ✅ **Path Approval Service** - Centralized path coordination to prevent conflicts
- ✅ **Dynamic Path Replanning** - Automatic rerouting when robots are blocked
- ✅ **Priority-Based Conflict Resolution** - Distance-to-goal prioritization system
- ✅ **Deadlock Detection & Recovery** - Automatic timeout-based deadlock resolution
- ✅ **Round-Robin Task Allocation** - Fair distribution of excavation points
- ✅ **Per-Robot Charging Stations** - Dedicated charging locations prevent conflicts
- ✅ **Unity-ROS2 Integration** - Seamless bi-directional communication

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         UNITY SIMULATION                             │
│                                                                       │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │          GenericRobotController (Base Class)                   │ │
│  │  • Movement & Pathfinding  • Battery Management References    │ │
│  │  • ROS Communication       • Charging Station Logic           │ │
│  │  • Collision Response      • Dynamic Path Replanning          │ │
│  └──────────────────────────┬────────────────────────────────────┘ │
│                             │ Inherits                              │
│  ┌──────────────────────────▼──────────┐  ┌──────────────────────┐ │
│  │     ExplorerController               │  │  BatterySimulator    │ │
│  │                                      │  │                      │ │
│  │ • Mission Queue & Planning           │  │ • Drain Physics      │ │
│  │ • Cost Calculation                   │  │ • Charge Physics     │ │
│  │ • Excavation Point Management        │  │ • ROS Publishing     │ │
│  │ • Round-Robin Task Allocation        │  │                      │ │
│  └──────────────────────────────────────┘  └──────────────────────┘ │
│           │                                        │                 │
│           │ Requests Paths                         │ Publishes       │
│           │ Publishes Position                     ▼                 │
│           ▼                                                          │
└───────────────────────────────────────────────────────────────────────┘
            │                      │                      │
            │ /target              │ /battery_state       │ /pose
            │ /astar_path          │ /charging_status     │ /collision_detected
            ▼                      ▼                      ▼
┌───────────────────────────────────────────────────────────────────────┐
│                         ROS2 SYSTEM                                   │
│                                                                       │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │              Collision Coordinator Node                        │  │
│  │  • Real-Time Collision Detection  • Path Approval Service     │  │
│  │  • Predictive Algorithm (1s)      • Priority Resolution       │  │
│  │  • Deadlock Detection (8s)        • Hysteresis Control (0.5s) │  │
│  └───────────────────────────────────────────────────────────────┘  │
│                                                                       │
│  ┌────────────────────┐         ┌──────────────────────────────┐   │
│  │ A* Navigation Node │         │    Battery Manager Node      │   │
│  │                    │         │                              │   │
│  │ • Path Planning    │         │ • Health Monitoring          │   │
│  │ • Obstacle Avoid   │         │ • Drain Rate Calculation     │   │
│  │ • Grid Conversion  │         │ • Alert System               │   │
│  │ • Service Client   │         │ • Metrics Publishing         │   │
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
│   │   │   ├── GenericRobotController.cs # Base class for all robot types
│   │   │   ├── ExplorerController.cs     # Explorer-specific mission planning
│   │   │   ├── BatterySimulator.cs       # Physics-based battery simulation
│   │   │   ├── AStarController.cs        # Unity A* integration
│   │   │   └── MapGenerator.cs           # Occupancy grid generation
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

#### 1. GenericRobotController (Base Class)
**Purpose**: Shared functionality for all robot types

**Responsibilities**:
- Movement along paths with rotation
- ROS2 communication (publishing targets, receiving paths)
- Battery management references (BatterySimulator, charging station)
- Path execution and waypoint following
- Abstract target reached handler for derived classes

**Key Parameters**:
- Linear Speed: `4.0 m/s`
- Angular Speed: `180 deg/s`
- Reach Threshold: `0.5 m`
- Charging Station Position: Configurable per robot type
- Battery Simulator Reference: Linked in Inspector
- Charged Threshold: `95%`
- Safety Cost Multiplier: `1.2` (20% safety margin)

#### 2. ExplorerController (Inherits from GenericRobotController)
**Purpose**: Explorer-specific mission planning

**Responsibilities**:
- Manages excavation point queue
- Calculates mission costs (target + return to charging)
- Decides when to insert charging station visits
- Implements continuous operation mode
- Overrides OnReachedTarget() for mission-specific behavior

**Explorer-Specific Parameters**:
- Charging Station Position: `(12, 0, -38)`
- Continuous Operation: `true` (continuous mission cycles)

#### 3. BatterySimulator
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

#### 4. Charging Zone
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

### Per-Robot Topics

Each robot (tb3_0 through tb3_5) has its own set of topics:

#### Battery Topics

| Topic Pattern | Type | Direction | Rate | Description |
|---------------|------|-----------|------|-------------|
| `/tb3_{i}/battery_state` | `std_msgs/Float32` | Unity → ROS2 | ~60 Hz | Current battery percentage (0-100) |
| `/tb3_{i}/charging_status` | `std_msgs/Bool` | Unity → ROS2 | On change | Charging state (true/false) |
| `/tb3_{i}/battery_health` | `std_msgs/String` | ROS2 | 5s | Battery health metrics (JSON) |
| `/tb3_{i}/battery_alert` | `std_msgs/String` | ROS2 | On event | Battery alerts (JSON) |

#### Navigation Topics

| Topic Pattern | Type | Direction | Rate | Description |
|---------------|------|-----------|------|-------------|
| `/tb3_{i}/target` | `geometry_msgs/PoseArray` | Unity → ROS2 | On demand | [robot_position, target_position] |
| `/tb3_{i}/astar_path` | `nav_msgs/Path` | ROS2 → Unity | On demand | Computed waypoint path |

#### Collision Avoidance Topics

| Topic Pattern | Type | Direction | Rate | Description |
|---------------|------|-----------|------|-------------|
| `/tb3_{i}/pose` | `geometry_msgs/PoseStamped` | Unity → ROS2 | 10 Hz | Robot position for collision detection |
| `/tb3_{i}/collision_detected` | `std_msgs/Bool` | ROS2 → Unity | 10 Hz | Emergency stop signal |

### Shared Topics

These topics are shared across all robots:

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Unity → ROS2 | Once | Static occupancy grid for pathfinding |
| `/robots` | `geometry_msgs/PoseArray` | Unity → ROS2 | Once | Initial robot spawn positions |
| `/targets` | `geometry_msgs/PoseArray` | Unity → ROS2 | Once | Excavation point positions |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/request_path_approval` | `space_project/PathApproval` | Request approval before planning path (multi-robot coordination) |

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

#### GenericRobotController (Base Class)
```
Movement Settings:
├─ Linear Speed: 4.0 m/s
├─ Angular Speed: 180 deg/s
├─ Reach Threshold: 0.5 m
└─ Robot Id: Set per robot type

Battery Management:
├─ Battery Simulator: Reference to BatterySimulator component
├─ Charging Station Position: Set per robot type
├─ Charged Threshold: 95
└─ Safety Cost Multiplier: 1.2

ROS Topics:
├─ Topic Name Target: "/target"
└─ Topic Name Path: "/astar_path"
```

#### ExplorerController (Inherits from GenericRobotController)
```
Mission Behavior:
└─ Continuous Operation: true (enables continuous mission cycles)

Note: Movement, Battery Management, and ROS Topics are inherited from GenericRobotController
      and configured in the base class inspector section.
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
3. **Reusability**: GenericRobotController enables multiple robot types with shared functionality
4. **Scalability**: Easy to add new robot types by inheriting from GenericRobotController
5. **Maintainability**: Changes to shared behavior only need updates in base class

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

## Multi-Robot Coordination System

### Overview

This system implements sophisticated multi-robot coordination using a **hybrid approach** combining real-time collision detection (reactive) and path approval services (predictive) to enable safe, efficient multi-robot operation.

### Coordination Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    HYBRID COORDINATION                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  PREDICTIVE (Before Movement)         REACTIVE (During Movement) │
│  ┌─────────────────────────┐         ┌──────────────────────┐  │
│  │ Path Approval Service   │         │ Collision Detection  │  │
│  │                         │         │                      │  │
│  │ • Goal conflict check   │         │ • Real-time tracking │  │
│  │ • Priority evaluation   │         │ • Velocity-based     │  │
│  │ • Async approval/reject │         │   prediction (1s)    │  │
│  │ • 3s retry backoff      │         │ • 10Hz publish rate  │  │
│  └─────────────────────────┘         └──────────────────────┘  │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

### 1. Collision Coordinator Node

**Purpose:** Centralized collision avoidance and path approval for all robots

**File:** `space_project/collision_coordinator_node.py`

**Responsibilities:**
- Real-time collision detection using robot positions and velocities
- Predictive collision detection (1-second prediction horizon)
- Path approval service for pre-movement coordination
- Priority-based conflict resolution
- Deadlock detection and timeout recovery
- Hysteresis control to prevent state oscillation

**Key Parameters:**
```python
num_robots = 6                          # Number of robots (1-6)
safe_distance = 4.0                     # Minimum separation (meters)
path_buffer = 3.0                       # Planning buffer zone (meters)
collision_warning_distance = 6.0        # Early warning threshold (meters)
deadlock_timeout = 8.0                  # Deadlock detection timeout (seconds)
hysteresis_duration = 0.5               # State change cooldown (seconds)
prediction_time = 1.0                   # Collision prediction horizon (seconds)
```

**Subscribed Topics:**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/tb3_{i}/pose` | `PoseStamped` | 10 Hz | Robot position & orientation |

**Published Topics:**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/tb3_{i}/collision_detected` | `Bool` | 10 Hz | Emergency stop signal per robot |

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/request_path_approval` | `PathApproval` | Approve/reject path requests before planning |

#### PathApproval Service Definition

**File:** `space_project/srv/PathApproval.srv`

**Request:**
```
int32 robot_id              # Requesting robot index
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

### 2. Collision Detection Algorithm

#### Predictive Collision Detection

**Algorithm Flow:**
1. Receive position updates from all robots (10 Hz)
2. Calculate velocities from position deltas: `v = Δpos / Δtime`
3. Predict future positions: `pred_pos = current_pos + velocity × prediction_time`
4. Check both current AND predicted distances
5. Use minimum of (current_distance, predicted_distance) for collision decision

**Code Reference:** `collision_coordinator_node.py:105-178`

**Example:**
```
Robot A: pos=(10, 20), velocity=(2, 0) → predicted=(12, 20)
Robot B: pos=(15, 20), velocity=(-2, 0) → predicted=(13, 20)

Current distance: 5m (safe)
Predicted distance: 1m (collision!)
Decision: COLLISION IMMINENT - lower priority robot stops
```

**Benefits:**
- Prevents collisions before they occur
- Accounts for robot momentum and trajectory
- Smoother coordination than reactive-only systems

#### Priority-Based Conflict Resolution

**Priority Formula:**
```python
priority = distance_to_goal + (robot_id × 0.01)
```
- **Lower value** = **higher priority** (robot closer to goal)
- Robot ID tiebreaker ensures deterministic resolution
- Only the **lower-priority** robot stops

**Example:**
```
Robot 0: 15.0m from goal → priority = 15.00
Robot 2: 8.0m from goal → priority = 8.02
Collision detected → Robot 0 stops (higher priority value)
```

**Code Reference:** `collision_coordinator_node.py:235-246, 148-166`

#### Deadlock Detection

**Problem:** Two robots blocking each other indefinitely

**Solution:**
- Track how long each robot has been stopped (`robot_stopped_times`)
- If stopped > `deadlock_timeout` (8.0 seconds):
  - Clear collision flag to allow movement
  - Log `⚠️ DEADLOCK` warning
  - Robot attempts to continue (may trigger reroute)

**Code Reference:** `collision_coordinator_node.py:180-200`

**Synced with Unity:** GenericRobotController also has 8.0s timeout for path replanning

#### Hysteresis Control

**Problem:** Collision state oscillating rapidly (flapping)

**Solution:**
- Track timestamp of last state change per robot
- Only allow state change if ≥ `hysteresis_duration` (0.5s) has passed
- Prevents: STOP → RESUME → STOP → RESUME oscillation

**Code Reference:** `collision_coordinator_node.py:202-232`

**Example:**
```
Time 0.0s: Collision detected → STOP
Time 0.2s: Distance increased → (state change blocked by hysteresis)
Time 0.3s: Distance decreased → (still blocked)
Time 0.6s: Distance increased → RESUME (0.6s > 0.5s threshold)
```

### 3. Path Approval Service Workflow

**Purpose:** Prevent conflicts before robots start moving (predictive coordination)

#### Request Flow

1. **Robot publishes target** → `/{robot_name}/target` topic
2. **astar_navigation receives** target request
3. **Service availability check** (non-blocking, 0.1s timeout):
   - If available: Call approval service
   - If unavailable: Fall back to immediate planning (graceful degradation)
4. **Collision coordinator evaluates:**
   - Check goal distance from other robots' current positions
   - Check goal distance from other robots' stored goals
   - Compare priorities (distance-to-goal)
5. **Response:**
   - **Approved**: `approved=true, wait_time=0.0, reason="Path clear"`
   - **Rejected**: `approved=false, wait_time=3.0, reason="Wait for tb3_2 (5.2m away)"`
6. **astar_navigation handles response:**
   - If approved: Compute and publish path
   - If rejected: Retry after 3.0 seconds (unlimited attempts)

**Code References:**
- Coordinator: `collision_coordinator_node.py:248-324`
- Navigator integration: `astar_navigation_node.py:292-378`

#### Non-Blocking Service Integration

**Key Design Decision:** Service calls are **asynchronous** to prevent blocking

**Benefits:**
- Navigator continues processing other robot requests
- No deadlock if service is slow
- Graceful fallback if service unavailable
- Retry logic with constant 3-second backoff

**Implementation:**
```python
# Non-blocking check
if not self.path_approval_client.service_is_ready():
    # Fall back to immediate planning
    path = self.a_star_search(src, dest)
    self.publish_path(path, robot_idx)
    return

# Async call with callback
future = self.path_approval_client.call_async(request)
future.add_done_callback(lambda f: self.handle_response(f, robot_idx, src, dest))
```

**Code Reference:** `astar_navigation_node.py:292-378`

### 4. Dynamic Path Replanning

**Purpose:** Reroute robots when blocked for extended periods

**Trigger:** Robot stopped by collision for > 8.0 seconds

**Strategy:**
1. **Attempts 1-3:** Try lateral offset path (5.0m perpendicular)
2. **Attempts 4+:** Retry original goal
3. **Backoff delays:** 2s, 4s, 6s, 10s (exponential, capped at 10s)

**Code Reference:** `GenericRobotController.cs:114-154`

**Example Flow:**
```
t=0s: Robot A blocked by Robot B
t=8s: Timeout → Request offset path (5m lateral)
t=10s: Retry after 2s delay (attempt 1)
t=18s: Timeout again → Request offset path again
t=22s: Retry after 4s delay (attempt 2)
t=30s: Timeout again → Request offset path again
t=36s: Retry after 6s delay (attempt 3)
t=44s: Timeout again → Retry ORIGINAL goal
t=54s: Retry after 10s delay (attempt 4+)
```

**Lateral Offset Calculation:**
```csharp
// 5 meters perpendicular to forward direction
Vector3 lateralOffset = Vector3.Cross(Vector3.up, transform.forward).normalized * 5f;
Vector3 offsetGoal = currentTarget.Position + lateralOffset;
```

### 5. Task Allocation System

#### Round-Robin Distribution Algorithm

**Purpose:** Fair distribution of excavation points across robots

**Algorithm:**
```csharp
// Sort points for deterministic assignment
Array.Sort(allPoints, (a, b) => {
    int xCompare = a.position.x.CompareTo(b.position.x);
    if (xCompare != 0) return xCompare;
    return a.position.z.CompareTo(b.position.z);
});

// Assign every Nth point to robot N
for (int i = 0; i < allPoints.Length; i++) {
    if (i % totalRobots == robotIndex) {
        excavationPointsList.Add(allPoints[i]);
    }
}
```

**Code Reference:** `ExplorerController.cs:163-183`

**Example (8 points, 4 robots):**
```
Sorted points: [P0, P1, P2, P3, P4, P5, P6, P7]
Robot 0 gets: P0, P4 (indices 0, 4)
Robot 1 gets: P1, P5 (indices 1, 5)
Robot 2 gets: P2, P6 (indices 2, 6)
Robot 3 gets: P3, P7 (indices 3, 7)
```

**Configuration:** Set `numberOfExcavationPoints` in MapGenerator Inspector (Range: 1-20, default: 8)

#### Per-Robot Charging Stations

**Purpose:** Prevent charging station conflicts

**Configuration:**
```csharp
// MapGenerator.cs: Predefined charging points
private Vector3[] chargingPoints = new Vector3[] {
    new Vector3(42f, 0f, -38f),  // Robot 0
    new Vector3(37f, 0f, -38f),  // Robot 1
    new Vector3(32f, 0f, -38f),  // Robot 2
    new Vector3(27f, 0f, -38f),  // Robot 3
    new Vector3(22f, 0f, -38f),  // Robot 4
    new Vector3(17f, 0f, -38f),  // Robot 5
};
```

**Code Reference:** `MapGenerator.cs:89-97`

**Assignment:** Robot N spawns at charging point N and returns there for charging

**Spacing:** 5 meters apart along X-axis prevents collision during charging missions

### 6. Staggered Robot Initialization

**Problem:** Simultaneous topic registration from multiple robots crashes ROS TCP endpoint

**Example:**
- 6 robots × 4 topics each = 24+ simultaneous registrations
- ROS TCP endpoint overloaded and crashes

**Solution:** Stagger initialization by robot index

```csharp
float staggerDelay = robotIndex * 0.5f;  // 0s, 0.5s, 1.0s, 1.5s, 2.0s, 2.5s
yield return new WaitForSeconds(staggerDelay);
```

**Code Reference:** `ExplorerController.cs:40-45`

**Benefits:**
- Prevents endpoint crashes
- Gradual topic registration
- Reliable multi-robot startup

### 7. Complete Topic Reference

#### Per-Robot Topics

Each robot (tb3_0 through tb3_5) has the following topics:

| Topic Pattern | Type | Direction | Rate | Description |
|---------------|------|-----------|------|-------------|
| `/tb3_{i}/pose` | `PoseStamped` | Unity → ROS2 | 10 Hz | Position for collision detection |
| `/tb3_{i}/target` | `PoseArray` | Unity → ROS2 | On demand | Path request [robot_pos, target_pos] |
| `/tb3_{i}/astar_path` | `Path` | ROS2 → Unity | On demand | Computed waypoint path |
| `/tb3_{i}/collision_detected` | `Bool` | ROS2 → Unity | 10 Hz | Emergency stop signal |
| `/tb3_{i}/battery_state` | `Float32` | Unity → ROS2 | 60 Hz | Battery level (0-100%) |
| `/tb3_{i}/charging_status` | `Bool` | Unity → ROS2 | On change | Charging state |
| `/tb3_{i}/battery_health` | `String` (JSON) | ROS2 | 5s | Battery metrics |
| `/tb3_{i}/battery_alert` | `String` (JSON) | ROS2 | On event | Battery alerts |

#### Shared Topics

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/map` | `OccupancyGrid` | Unity → ROS2 | Once | Static map for pathfinding |
| `/robots` | `PoseArray` | Unity → ROS2 | Once | Initial robot positions |
| `/targets` | `PoseArray` | Unity → ROS2 | Once | Excavation point positions |

### 8. Configuration Parameters

#### Unity Inspector (MapGenerator)

```
Spawning Options:
├─ Number Of Robots: 0-6 (0 = random)
└─ Number Of Excavation Points: 1-20 (default: 8)

Recommended: excavationPoints ≥ 2 × numRobots for mission variety
```

#### ROS2 Launch Parameters

**File:** `space_project/launch/battery_system.launch.py`

```python
# Collision Coordination
num_robots = 6                          # Number of robots
safe_distance = 4.0                     # Minimum separation (meters)
path_buffer = 3.0                       # Planning buffer (meters)
collision_warning_distance = 6.0        # Early warning (meters)

# Battery Management
robot_name = 'tb3_0'                    # Robot identifier
low_battery_threshold = 30.0            # Low battery warning (%)
critical_battery_threshold = 15.0       # Critical alert (%)
charged_threshold = 95.0                # Charging complete (%)
publish_metrics_interval = 5.0          # Health metrics rate (seconds)
```

### 9. Multi-Robot Startup Sequence

**Step 1: Start ROS2 System**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch space_project battery_system.launch.py
```

**Expected Output:**
```
[INFO] [collision_coordinator_node]: Collision Coordinator started: 6 robots, safe_distance=4.0m
[INFO] [collision_coordinator_node]: Service /request_path_approval ready
[INFO] [astar_navigation]: Subscribed to /tb3_0/target ... /tb3_5/target
[INFO] [astar_navigation]: Path approval service connected!
[INFO] [battery_manager]: Battery monitoring started for tb3_0
```

**Step 2: Start Unity**
1. Press Play in Unity Editor
2. Robots spawn with staggered delays (0.5s each)
3. Each robot waits for ROS connection before registering topics

**Expected Unity Console:**
```
tb3_0: Waiting 0.0s before initialization
tb3_1: Waiting 0.5s before initialization (stagger to prevent endpoint crash)
tb3_2: Waiting 1.0s before initialization (stagger to prevent endpoint crash)
...
tb3_0: ROS connected! Registering topics...
tb3_0: Initialization complete - ready to publish!
tb3_0: Charging station set to (42.0, 0.0, -38.0)
tb3_0: Assigned 2/8 excavation points (robot 0/4)
```

**Step 3: Verify System**
```bash
# Check all nodes running
ros2 node list
# Expected: /collision_coordinator_node, /astar_navigation, /battery_manager, /ros_tcp_endpoint

# Check collision topics
ros2 topic list | grep collision
# Expected: /tb3_0/collision_detected ... /tb3_5/collision_detected

# Monitor collision coordinator
ros2 topic echo /tb3_0/collision_detected
```

### 10. Expected Multi-Robot Behavior

#### Scenario 1: Path Approval (Predictive)

**Unity Console:**
```
tb3_2: PoseArray published Robot: (10.0, 0.0, 20.0) | Target: (30.0, 0.0, 40.0)
tb3_2: Starting mission to ExcavationPointTarget. Battery: 85.0% | Cost: 25.0%
```

**ROS2 Terminal:**
```
[INFO] [astar_navigation]: tb3_2: Path request from (10.0, 20.0) to (30.0, 40.0)
[INFO] [astar_navigation]: tb3_2: Path APPROVED - Path clear
[INFO] [astar_navigation]: tb3_2: Published path with 47 waypoints
```

**Unity Console:**
```
Path received from ROS. Length: 47
```

#### Scenario 2: Path Rejection (Conflict)

**Unity Console:**
```
tb3_1: PoseArray published Robot: (15.0, 0.0, 25.0) | Target: (32.0, 0.0, 41.0)
tb3_1: Starting mission to ExcavationPointTarget. Battery: 90.0% | Cost: 22.0%
```

**ROS2 Terminal:**
```
[INFO] [astar_navigation]: tb3_1: Path request from (15.0, 25.0) to (32.0, 41.0)
[WARN] [collision_coordinator_node]: ✗ tb3_1 REJECTED - conflict with tb3_2
[WARN] [astar_navigation]: tb3_1: Path NOT approved - Wait for tb3_2 (2.5m away). Retrying in 3.0s (attempt #1)
[INFO] [astar_navigation]: tb3_1: Retrying path request (attempt #1)
[INFO] [collision_coordinator_node]: ✓ tb3_1 APPROVED (priority over tb3_2)
[INFO] [astar_navigation]: tb3_1: Published path with 52 waypoints
```

#### Scenario 3: Collision Detection (Reactive)

**ROS2 Terminal:**
```
[WARN] [collision_coordinator_node]: COLLISION ALERT: tb3_0 and tb3_3 current: 3.8m, predicted: 2.1m (min safe: 4.0m)
[INFO] [collision_coordinator_node]: tb3_3: STOP
```

**Unity Console:**
```
tb3_3: COLLISION DETECTED - STOPPING
[Waits 8 seconds]
tb3_3: Waited 8.0s - requesting alternate path
tb3_3: Attempt 1 - trying offset path to (35.0, 0.0, 25.0)
```

**ROS2 Terminal:**
```
[WARN] [collision_coordinator_node]: ⚠️ DEADLOCK: tb3_3 stopped for 8.1s - clearing collision flag
[INFO] [collision_coordinator_node]: tb3_3: RESUME
```

**Unity Console:**
```
tb3_3: Path clear - RESUMING
```

### 11. Monitoring Multi-Robot System

**Terminal 1: Collision Status**
```bash
# Watch collision detections
ros2 topic echo /tb3_0/collision_detected
```

**Terminal 2: Position Tracking**
```bash
# Monitor robot positions
ros2 topic echo /tb3_0/pose
```

**Terminal 3: Path Approvals**
```bash
# Watch service calls
ros2 service call /request_path_approval space_project/srv/PathApproval "{robot_id: 0, start_x: 10.0, start_y: 20.0, goal_x: 30.0, goal_y: 40.0}"
```

**Terminal 4: System Graph**
```bash
# Visualize topic connections
ros2 run rqt_graph rqt_graph
```

### 12. Troubleshooting Multi-Robot Issues

#### Issue: Robots colliding despite collision avoidance

**Check:**
```bash
# Is collision coordinator running?
ros2 node list | grep collision

# Are collision topics being published?
ros2 topic list | grep collision_detected

# Are robots receiving collision signals?
ros2 topic echo /tb3_0/collision_detected
```

**Solution:**
1. Verify collision_coordinator_node is running
2. Check `safe_distance` parameter (increase if needed)
3. Verify Unity robots subscribe to collision topics

#### Issue: Path approval always rejected

**Check:**
```bash
# Monitor service responses
ros2 service call /request_path_approval space_project/srv/PathApproval "{robot_id: 0, start_x: 10.0, start_y: 20.0, goal_x: 30.0, goal_y: 40.0}"
```

**Solution:**
1. Check if goals are too close together (< `path_buffer` = 3.0m)
2. Increase `path_buffer` parameter if needed
3. Verify priority calculation is working

#### Issue: Deadlock not resolving

**Solution:**
1. Check `deadlock_timeout` matches Unity's `maxWaitTime` (both 8.0s)
2. Increase timeout if robots need more time
3. Verify path replanning logic in GenericRobotController

#### Issue: Robots all assigned same excavation points

**Check:**
```
Unity Console: "tb3_0: Assigned 4/4 excavation points (robot 0/4)"
```

**Solution:**
1. Set `numberOfExcavationPoints` in MapGenerator Inspector
2. Recommended: excavationPoints ≥ 2 × numRobots
3. Example: 8 points for 4 robots = 2 points each

#### Issue: All robots going to same charging station

**Check:**
```
Unity Console: All robots log same charging station coordinates
```

**Solution:**
1. Verify MapGenerator sets `chargingStationPosition` per robot (line 301)
2. Check each robot spawns at different position from `chargingPoints` array
3. Verify ExplorerController uses assigned position (not hardcoded)

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

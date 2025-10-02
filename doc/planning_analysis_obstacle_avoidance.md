# Autoware Planning Module Analysis: Obstacle Avoidance for Broken-Down Vehicles

**Author:** Analysis for BT-based Obstacle Avoidance Scenario Selection
**Focus:** Understanding current FSM-based approach for broken-down vehicle scenarios
**Date:** 2025-10-02

---

## 1. Executive Summary

This document analyzes Autoware's planning module architecture, focusing on obstacle avoidance mechanisms for broken-down vehicles (static obstacles). The current system uses **Finite State Machines (FSM)** for decision-making, which is the target for replacement with **Behavior Trees (BT)** in this thesis work.

### Key Findings:
- Autoware uses a **hierarchical planning architecture** with parallel scenario pipelines
- Obstacle avoidance for static objects (broken-down vehicles) is handled by **rule-based modules** with FSM state management
- Multiple FSM implementations exist across different modules
- Target areas for BT implementation: behavior module decision logic (NOT scenario selector itself)

### Three Target Scenarios for BT Implementation:

**1. Broken-Down Vehicle Scenario (Static Obstacle Avoidance)**
- **Module:** `autoware_behavior_path_static_obstacle_avoidance_module` (Behavior Path Planner)
- **Current Status:** ENABLED by default
- **Detection Criteria:** Object velocity < 1.0 m/s for > 2.0 seconds
- **Behavior:** Lateral shift path generation with safety validation
- **FSM States:** IDLE → RUNNING → SUCCESS/FAILURE

**2. Crosswalk Scenario**
- **Module:** `autoware_behavior_velocity_crosswalk_module` (Behavior Velocity Planner)
- **Current Status:** ENABLED by default
- **Detection:** Pedestrians within 2.0m of crosswalk
- **Behavior:** Stop at stop line or slow approach based on pedestrian TTC
- **Complex Logic:** Pass-judge FSM (ego-pass-first vs pedestrian-pass-first)

**3. Blind Corner Scenario (Occlusion Spot)**
- **Module:** `autoware_behavior_velocity_occlusion_spot_module` (Behavior Velocity Planner)
- **Current Status:** DISABLED by default (**must enable for thesis**)
- **Detection:** Occupancy grid-based occlusion detection
- **Behavior:** Slow down to safe velocity assuming pedestrian may dash at 1.5 m/s
- **Assumptions:** Pedestrian radius 0.3m, minimum occlusion size 1.0m²

---

## 2. Planning Module Architecture Overview

### 2.1 Directory Structure

The planning system is located at: `/home/peeradon/autoware/src/universe/autoware_universe/planning/`

Key components:
```
planning/
├── autoware_scenario_selector/         # High-level scenario switching (FSM-based)
├── behavior_path_planner/              # Path planning with avoidance modules
│   ├── autoware_behavior_path_static_obstacle_avoidance_module/
│   ├── autoware_behavior_path_dynamic_obstacle_avoidance_module/
│   └── autoware_behavior_path_planner_common/
├── behavior_velocity_planner/          # Velocity planning modules
│   ├── autoware_behavior_velocity_run_out_module/    # Contains FSM implementation
│   ├── autoware_behavior_velocity_crosswalk_module/
│   └── autoware_behavior_velocity_intersection_module/
├── motion_velocity_planner/            # Motion-level velocity planning
└── autoware_path_optimizer/            # Path optimization
```

### 2.2 Planning Hierarchy (CORRECTED)

**IMPORTANT:** The planning flow operates **scenario pipelines in parallel**, then the Scenario Selector **chooses the output**.

```
┌──────────────────────────────────────────────────────────────────────┐
│                    Mission Planning                                  │
│  - Route planning from start to goal                                 │
└────────────────────────────────┬─────────────────────────────────────┘
                                 ↓
    ┌────────────────────────────┴────────────────────────────┐
    │                                                          │
    ↓                                                          ↓
┌─────────────────────────────────┐        ┌─────────────────────────────────┐
│  LaneDriving Scenario Pipeline  │        │   Parking Scenario Pipeline     │
│ ┌─────────────────────────────┐ │        │ ┌─────────────────────────────┐ │
│ │  Behavior Path Planning     │ │        │ │  Freespace Planner          │ │
│ │  - lane_following           │ │        │ └─────────────────────────────┘ │
│ │  - static_obstacle_avoidance│ │        │               ↓                 │
│ │  - lane_change              │ │        │ ┌─────────────────────────────┐ │
│ │  - goal_planner             │ │        │ │  Trajectory Generation      │ │
│ └─────────────────────────────┘ │        │ └─────────────────────────────┘ │
│               ↓                  │        │               ↓                 │
│ ┌─────────────────────────────┐ │        │         Output Parking          │
│ │  Behavior Velocity Planning │ │        │         Trajectory              │
│ │  - crosswalk                │ │        └─────────────────────────────────┘
│ │  - intersection             │ │                       │
│ │  - traffic_light            │ │                       │
│ │  - run_out                  │ │                       │
│ └─────────────────────────────┘ │                       │
│               ↓                  │                       │
│ ┌─────────────────────────────┐ │                       │
│ │  Motion Planning            │ │                       │
│ │  - obstacle_avoidance       │ │                       │
│ │  - obstacle_velocity_limiter│ │                       │
│ └─────────────────────────────┘ │                       │
│               ↓                  │                       │
│         Output LaneDriving       │                       │
│         Trajectory               │                       │
└─────────────────┬────────────────┘                       │
                  │                                        │
                  └────────────┬───────────────────────────┘
                               ↓
                  ┌────────────────────────────┐
                  │   Scenario Selector (FSM)  │ ← Selects trajectory
                  │  Inputs:                   │   based on scenario state
                  │  - LaneDriving trajectory  │
                  │  - Parking trajectory      │
                  │  - Current scenario state  │
                  └────────────┬───────────────┘
                               ↓
                  ┌────────────────────────────┐
                  │  Motion Velocity Smoother  │ ← Final smoothing
                  └────────────┬───────────────┘
                               ↓
                  ┌────────────────────────────┐
                  │   Planning Validator       │ ← Safety validation
                  └────────────┬───────────────┘
                               ↓
                          To Control
```

**Key Insights:**
1. **LaneDriving and Parking scenarios run in parallel** as separate pipelines
2. **Behavior modules (avoidance, crosswalk, etc.) run INSIDE scenario pipelines**, NOT after scenario selector
3. **Scenario Selector's role:** Select which scenario's output trajectory to use (NOT to generate paths itself)
4. **For broken-down vehicle avoidance:** Happens in LaneDriving pipeline's Behavior Path Planner

### 2.3 Module Execution Hierarchy and Configuration

**Module Priority Slots** (from `scene_module_manager.param.yaml`):
- **Slot 1 (Highest Priority):** `start_planner` - Handles vehicle startup from parking
- **Slot 2 (Main Behaviors):** Multiple modules can run simultaneously:
  - `static_obstacle_avoidance` ← **Target for BT replacement (broken-down vehicles)**
  - `lane_change_left/right` - Lane change behaviors
  - `avoidance_by_lane_change` - Alternative avoidance strategy
  - `bidirectional_traffic` - Opposite lane handling
- **Slot 3:** `goal_planner` - Handles arrival at destination
- **Slot 4 (Lowest Priority):** `dynamic_obstacle_avoidance` - Currently disabled by default

**Important Configuration Findings:**
1. **Static obstacle avoidance is ENABLED by default** (`launch_static_obstacle_avoidance: true`)
2. **Crosswalk module is ENABLED** (`launch_crosswalk_module: true`)
3. **Occlusion spot (blind corner) is DISABLED by default** (`launch_occlusion_spot_module: false`)
   - **⚠️ Need to enable this for thesis testing!**
4. **Dynamic obstacle avoidance is DISABLED** (`launch_dynamic_obstacle_avoidance: false`)

**RTC (Request to Cooperate) Settings:**
- Most modules have `enable_rtc: false` by default (AUTO mode)
- Static obstacle avoidance: Can run without manual approval in normal cases
- Ambiguous vehicles: Policy set to "manual" (requires operator approval)

---

## 3. FSM Implementations in Current System

### 3.1 Scenario Selector FSM

**Location:** `autoware_scenario_selector/src/scenario_selector_node.cpp`

**Purpose:** High-level scenario switching between LaneDriving and Parking

**State Transitions:**
```
┌──────────┐
│  EMPTY   │ (Initial state)
└─────┬────┘
      ↓
  ┌───────────────┐
  │  LaneDriving  │ ←→ Parking
  └───────────────┘
```

**Transition Logic:**
- Empty → LaneDriving: Vehicle is in lane
- Empty → Parking: Vehicle is in parking lot & goal not in lane
- LaneDriving → Parking: In parking lot & goal not in lane
- Parking → LaneDriving: Parking completed & in lane

**Key Characteristics:**
- Simple binary state machine
- Scenario completion flag-based
- Position-based (in_lane check)

---

### 3.2 Module Status FSM (Common Framework)

**Location:** `behavior_path_planner_common/interface/scene_module_interface.hpp`

**All behavior modules inherit this status system:**

```cpp
enum class ModuleStatus {
  IDLE = 0,              // Module not active
  RUNNING = 1,           // Module executing
  WAITING_APPROVAL = 2,  // Waiting for RTC approval (manual mode)
  SUCCESS = 3,           // Module completed successfully
  FAILURE = 4,           // Module failed
};
```

**Application:** Used by ALL behavior path planning modules including:
- Static Obstacle Avoidance
- Dynamic Obstacle Avoidance
- Lane Change
- Goal Planner

---

### 3.3 Run Out Module State Machine (Example)

**Location:** `behavior_velocity_planner/autoware_behavior_velocity_run_out_module/src/state_machine.hpp`

**States:**
```cpp
enum class State {
  GO = 0,        // Normal driving
  STOP,          // Vehicle stopped
  APPROACH,      // Approaching obstacle cautiously
  UNKNOWN,       // Error state
};
```

**Transition Logic:**
```
┌────┐
│ GO │ ←──────────────────────────┐
└─┬──┘                            │
  │ velocity < threshold          │
  ↓                               │
┌──────┐                          │
│ STOP │                          │
└─┬────┘                          │
  │ stopped > time_threshold      │
  ↓                               │
┌──────────┐                      │
│ APPROACH │──────────────────────┘
└──────────┘  dist > threshold or
              no obstacle
```

**Implementation Details:**
- Switch-case based state transitions
- Time-based conditions (stop duration)
- Distance-based conditions (distance to collision)
- Velocity-based conditions

**Code Reference:** [state_machine.cpp:42-109](../src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_run_out_module/src/state_machine.cpp)

---

## 4. Static Obstacle Avoidance for Broken-Down Vehicles

### 4.1 Module Overview

**Location:** `behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/`

**Purpose:** Rule-based avoidance planning for stopped/parked vehicles (including broken-down vehicles)

**Key Characteristics:**
- **Rule-based approach** (not learning-based)
- Handles **static obstacles only** (moving objects excluded)
- Creates avoidance paths by **lateral shifting**
- Uses **RTC (Request to Cooperate) interface** for manual approval
- Operates within lane constraints and traffic rules

### 4.2 Module Workflow

```
┌─────────────────────────────────────────────────────────┐
│ 1. updateData() - Fill Fundamental Data                │
│    - Reference path from centerline                     │
│    - Current lanelets & drivable bounds                 │
│    - Avoidance start/return points                      │
├─────────────────────────────────────────────────────────┤
│ 2. fillAvoidanceTargetObjects()                         │
│    - Filter objects by type, state, position            │
│    - Check: stopped, on lane edge, distance thresholds  │
├─────────────────────────────────────────────────────────┤
│ 3. fillShiftLine()                                      │
│    - Generate lateral shift trajectory                  │
│    - Safety check (collision with moving vehicles)      │
│    - Validate: jerk constraints, drivable area          │
├─────────────────────────────────────────────────────────┤
│ 4. fillEgoStatus() - Determine Module State            │
│    - getCurrentModuleState()                            │
│       → RUNNING / CANCEL / SUCCEEDED                    │
│    - Check yield maneuver necessity                     │
├─────────────────────────────────────────────────────────┤
│ 5. plan() - Generate Final Path                        │
│    - Add shift lines (if approved)                      │
│    - Insert velocity constraints                        │
│    - Insert wait points (if needed)                     │
│    - Calculate turn signals                             │
└─────────────────────────────────────────────────────────┘
```

### 4.3 Object Filtering Logic (Broken-Down Vehicle Detection)

**Conditions to identify avoidance targets:**

1. **Object Type Check**
   - Target types: CAR, TRUCK, BUS (configurable)

2. **Movement Check**
   - Object velocity < `moving_speed_threshold`
   - Stopped for > `moving_time_threshold` duration

3. **Position Checks**
   - Object is within detection area (`object_check_forward_distance`)
   - Object is on the edge of the lane (not centered)
   - Distance to centerline < threshold (`threshold_distance_object_is_on_center`)
   - Object is on road shoulder (using `object_check_shiftable_ratio`)

4. **Feasibility Checks**
   - Sufficient lateral space for avoidance
   - Not too close to goal position
   - Not in restricted areas (crosswalk vicinity, traffic light sections)

**Object Classification Reasons** (from `data_structs.hpp`):
```cpp
enum class ObjectInfo {
  // Avoidable
  NONE = 0,

  // Ignore reasons
  OUT_OF_TARGET_AREA,
  FURTHER_THAN_THRESHOLD,
  IS_NOT_TARGET_OBJECT,
  IS_NOT_PARKING_OBJECT,
  TOO_NEAR_TO_CENTERLINE,
  TOO_NEAR_TO_GOAL,
  MOVING_OBJECT,
  CROSSWALK_USER,
  ENOUGH_LATERAL_DISTANCE,

  // Unavoidable reasons (broken-down vehicle might fall here)
  NEED_DECELERATION,
  INSUFFICIENT_DRIVABLE_SPACE,
  INSUFFICIENT_LONGITUDINAL_DISTANCE,
  INVALID_SHIFT_LINE,

  // Ambiguous cases
  AMBIGUOUS_STOPPED_VEHICLE,
};
```

### 4.4 Avoidance Path Generation

**Method:** Path Shifter (lateral shifting)

1. **Shift Line Generation:**
   - Calculate required lateral offset (object width + safety margin)
   - Determine shift start/end points based on jerk constraints
   - Smooth shift trajectory using spline interpolation

2. **Safety Validation:**
   - Check collision with surrounding moving vehicles
   - Validate path is within drivable area
   - Ensure lateral jerk < max threshold
   - Verify sufficient longitudinal distance

3. **Velocity Planning:**
   - **Prepare velocity:** Slow down before shift starts (to meet jerk constraints)
   - **Avoidance velocity:** Speed control while passing object
   - **Wait points:** Insert stops if safety check fails or waiting for approval

### 4.5 Module State Management (FSM-like behavior)

**States** (derived from workflow):
```
RUNNING:   Target objects exist, avoidance path being executed
CANCEL:    Target objects gone before avoidance initiated
SUCCEEDED: All objects avoided, returned to original lane
YIELD:     Cannot avoid safely → stop and wait
```

**State Transitions:**
```
        ┌──────────┐
   ┌───→│ RUNNING  │────┐
   │    └──────────┘    │
   │                    │ Target gone
   │                    ↓  (before shift)
   │               ┌────────┐
   │               │ CANCEL │
   │               └────────┘
   │
   │    Unsafe
   ↓
┌───────┐           ┌───────────┐
│ YIELD │           │ SUCCEEDED │
└───────┘           └───────────┘
   ↑                     ↑
   │                     │
   └─────────────────────┘
     All avoided & returned
```

**Decision Points (where BT could improve):**
- Target object filtering (multiple conditions)
- Shift line validity checks
- Safety assessment (collision check)
- Yield maneuver triggering
- Return-to-lane timing

---

## 5. Dynamic Obstacle Avoidance (Complementary System)

**Location:** `behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/`

**Differences from Static Avoidance:**
- Handles **moving objects** (not relevant for broken-down vehicles)
- Works **within lanes** (cuts drivable area)
- Uses **predicted paths** and **TTC (Time To Collision)**
- Integrates with `autoware_path_optimizer` for trajectory optimization

**Note:** Broken-down vehicles are primarily handled by **static avoidance** module.

---

## 6. FSM Limitations & BT Opportunities

### 6.1 Current FSM Limitations

**1. Rigid State Transitions:**
- Switch-case based logic → hard to extend
- Limited ability to handle concurrent conditions
- Difficulty in representing hierarchical decisions

**2. Limited Modularity:**
- Object filtering logic embedded in monolithic functions
- Safety checks tightly coupled with path generation
- Hard to reuse logic across scenarios

**3. Poor Reactivity:**
- State transitions based on discrete checks
- Limited ability to handle partial goal achievement
- No built-in fallback mechanisms

**4. Scenario Complexity:**
- Broken-down vehicles + crosswalk + blind corner scenarios require complex FSM
- Combinatorial explosion of states
- Difficult to maintain and debug

### 6.2 BT Advantages for This Domain

**1. Modularity:**
```
BehaviorTree: AvoidBrokenDownVehicle
├── Sequence: DetectAndClassify
│   ├── IsObjectStatic?
│   ├── IsOnRoadEdge?
│   └── IsSafetyMarginOK?
├── Fallback: PlanAvoidance
│   ├── PlanLateralShift
│   └── PlanYieldManeuver
└── Sequence: ExecuteAvoidance
    ├── WaitForApproval [if manual mode]
    ├── ExecuteShift
    └── ReturnToLane
```

**2. Hierarchical Composition:**
- Crosswalk + Blind Corner + Broken Vehicle can be composed
- Parent BT can switch between scenario sub-trees
- Easy to add new scenarios

**3. Reactive Behavior:**
- Leaf nodes continuously evaluated
- Fallback nodes provide automatic recovery
- Decorator nodes (retry, timeout) built-in

**4. Maintainability:**
- Visual representation of logic
- Easy to identify bottlenecks
- Testable individual nodes

---

## 7. Proposed BT Integration Points

### 7.1 High Priority: Replace FSM in Behavior Modules (NOT Scenario Selector)

**CORRECTED UNDERSTANDING:**

The Scenario Selector simply picks between LaneDriving/Parking outputs - it's NOT where the complex decision logic happens.

**The real FSM-to-BT replacement targets are:**

**A) Behavior Path Planner Modules (Inside LaneDriving Pipeline)**

**Current:** Each module has ModuleStatus FSM (IDLE/RUNNING/WAITING_APPROVAL/SUCCESS/FAILURE)

**Proposed BT Structure for Static Obstacle Avoidance:**
```
Root: StaticObstacleAvoidanceModule (Sequence)
├── SubTree: ObjectDetectionAndFiltering
│   ├── Fallback: ClassifyObject
│   │   ├── Sequence: CheckIfAvoidable
│   │   │   ├── IsObjectStatic?
│   │   │   ├── IsOnLaneEdge?
│   │   │   ├── HasSufficientSpace?
│   │   │   └── Success
│   │   └── Failure (ignore object)
│   └── UpdateTargetObjects
├── SubTree: PathGeneration
│   ├── Fallback: GenerateAvoidancePath
│   │   ├── Sequence: LateralShiftPath
│   │   │   ├── CalculateShiftLine
│   │   │   ├── ValidateSafety
│   │   │   └── Success
│   │   └── Sequence: YieldManeuver
│   │       ├── CalculateStopPoint
│   │       └── WaitForClearance
├── Decorator[RTC]: WaitForApproval [if manual mode]
└── SubTree: ExecuteAndMonitor
    ├── ExecutePath
    ├── MonitorCompletion
    └── ReturnToLane
```

**B) Behavior Velocity Planner Modules**

**Proposed BT Structure for Crosswalk/Blind Corner Integration:**
```
Root: CrosswalkBehavior (Sequence)
├── DetectCrosswalk
├── Fallback: DecideAction
│   ├── Sequence: StopForPedestrian
│   │   ├── IsPedestrianOnCrosswalk?
│   │   ├── CalculateStopLine
│   │   └── InsertStopPoint
│   └── Sequence: SlowApproach
│       ├── IsPedestrianNearby?
│       └── ReduceVelocity
└── MonitorPassage
```

**Benefits:**
- Modules are composable (crosswalk + broken vehicle can both run)
- Each scenario (crosswalk, blind corner, avoidance) is independent BT
- Easy to add conditions and fallbacks
- Better than ModuleStatus FSM for complex state management

### 7.2 Medium Priority: Avoidance Module State Management

**Current:** ModuleStatus FSM (IDLE/RUNNING/WAITING_APPROVAL/SUCCESS/FAILURE)

**Proposed BT Structure:**
```
Root: StaticObstacleAvoidance (Sequence)
├── SubTree: TargetObjectFiltering
│   ├── Fallback: ObjectClassification
│   │   ├── IsMovingObject? → Failure
│   │   ├── IsTooNearGoal? → Failure
│   │   └── Success
│   └── Condition: IsAvoidanceRequired
├── SubTree: PathPlanning
│   ├── Fallback: GeneratePath
│   │   ├── PlanLateralShift
│   │   └── PlanYieldManeuver
│   └── Decorator[Retry(3)]: ValidateSafety
├── SubTree: Execution
│   ├── Sequence[if manual_mode]: WaitForApproval
│   └── Action: ExecutePath
└── Action: ReturnToLane
```

### 7.3 Low Priority: Object Filtering Logic

**Current:** Multiple if-statements for object classification

**Proposed:** Individual BT condition nodes
- `IsObjectTypeTarget`
- `IsObjectStopped`
- `IsObjectOnLaneEdge`
- `IsSufficientSpace`
- etc.

**Benefits:** Reusable across different avoidance scenarios

---

## 8. Implementation Roadmap for Thesis

### Phase 1: Analysis & Design (Current)
- ✅ Understand Autoware planning architecture
- ✅ Identify FSM implementations
- ✅ Document obstacle avoidance workflow
- ⬜ Design BT architecture for 3 scenarios

### Phase 2: BT Framework Integration
- ⬜ Select BT library (BehaviorTree.CPP recommended)
- ⬜ Integrate BT library with ROS 2 / Autoware
- ⬜ Create basic BT nodes (conditions, actions)
- ⬜ Build BT visualization/debugging tools

### Phase 3: Scenario Implementation
- ⬜ **Broken-Down Vehicle BT**
  - Replace static obstacle avoidance FSM logic
  - Implement object detection/classification nodes
  - Implement path planning nodes
  - Implement execution nodes

- ⬜ **Crosswalk BT**
  - Integrate with `behavior_velocity_crosswalk_module`
  - Pedestrian detection & prediction nodes
  - Stop line planning nodes

- ⬜ **Blind Corner BT**
  - Integrate with occlusion spot module
  - Slow down behavior nodes
  - Safety validation nodes

### Phase 4: Testing in CARLA
- ⬜ Setup Autoware-CARLA integration
- ⬜ Create test scenarios (broken-down vehicle, crosswalk, blind corner)
- ⬜ Compare FSM vs BT performance
  - Safety metrics
  - Reaction time
  - Path smoothness
  - Failure recovery

### Phase 5: Evaluation & Thesis Writing
- ⬜ Quantitative analysis (safety, efficiency)
- ⬜ Qualitative analysis (maintainability, extensibility)
- ⬜ Document findings

---

## 9. Key Files for Modification

### 9.1 Configuration Files (autoware_launch)

**Module Enable/Disable:**
- `/autoware_launch/config/planning/preset/default_preset.yaml`
  - Enable/disable planning modules globally
  - Key flags:
    - `launch_static_obstacle_avoidance: true` (currently enabled)
    - `launch_crosswalk_module: true` (currently enabled)
    - `launch_occlusion_spot_module: false` (currently disabled - blind corner)
    - `launch_dynamic_obstacle_avoidance: false` (currently disabled)

**Module Priority and Execution Order:**
- `/autoware_launch/config/planning/.../behavior_path_planner/scene_module_manager.param.yaml`
  - Defines module execution slots (priority order):
    ```yaml
    slot1: ["start_planner"]
    slot2: ["side_shift", "avoidance_by_lane_change", "static_obstacle_avoidance",
            "lane_change_left", "lane_change_right", "bidirectional_traffic"]
    slot3: ["goal_planner"]
    slot4: ["dynamic_obstacle_avoidance"]
    ```
  - RTC (Request to Cooperate) configuration per module
  - Simultaneous execution settings

**Static Obstacle Avoidance Parameters:**
- `/autoware_launch/config/planning/.../static_obstacle_avoidance.param.yaml`
  - Object type thresholds (car, truck, bus, etc.):
    - `th_moving_speed: 1.0 m/s` (objects slower = stopped)
    - `th_moving_time: 2.0 s` (must be stopped for this duration)
    - Lateral margins (soft: 0.5m, hard: 0.2m)
  - Target filtering:
    - `object_check_goal_distance: 20.0 m`
    - `th_offset_from_centerline: 1.0 m` (parked vehicle detection)
    - `th_shiftable_ratio: 0.8` (road shoulder width ratio)
  - Safety check configuration
  - Ambiguous vehicle policy: "manual" (requires operator approval)

**Crosswalk Parameters:**
- `/autoware_launch/config/planning/.../crosswalk.param.yaml`
  - `enable_rtc: false` (no manual approval by default)
  - Stop position thresholds
  - Pass judge logic (ego vs pedestrian priority)
  - Occlusion handling (disabled by default)

**Occlusion Spot (Blind Corner) Parameters:**
- `/autoware_launch/config/planning/.../occlusion_spot.param.yaml`
  - Detection method: "occupancy_grid"
  - Pedestrian assumptions: `pedestrian_vel: 1.5 m/s`, `pedestrian_radius: 0.3 m`
  - Detection area & slow down behavior

### 9.2 Source Code Files (autoware_universe)

**Static Obstacle Avoidance Module:**
- `behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/manager.cpp`
  - Module initialization & registration

- `behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`
  - Main avoidance logic (replace with BT executor)

- `behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/utils.cpp`
  - Object filtering functions (convert to BT condition nodes)

**Common Framework:**
- `behavior_path_planner_common/interface/scene_module_interface.hpp`
  - Module status enum (may need BT-compatible version)

**Crosswalk Module:**
- `behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene.cpp`

**Occlusion Spot Module:**
- `behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module/src/scene.cpp`

### 9.3 Launch Files

**Main Planning Launch:**
- `tier4_planning_launch/launch/planning.launch.xml`

**Scenario Planning Launch:**
- `tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml`

**LaneDriving Behavior Planning:**
- `tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml`

---

## 10. Technical Considerations

### 10.1 ROS 2 Integration
- Autoware uses **ROS 2 Humble**
- BT nodes need to be ROS 2 compatible
- Use ROS 2 services/topics for BT node communication

### 10.2 Real-Time Constraints
- Planning modules run at ~10 Hz
- BT tick rate must be compatible
- Avoid blocking operations in BT nodes

### 10.3 RTC Interface Compatibility
- Manual approval mode uses RTC (Request to Cooperate)
- BT must integrate with existing RTC framework
- Approval waits can be BT decorator nodes

### 10.4 Safety Validation
- Path safety checks are critical
- BT must maintain existing safety checks
- Consider safety validation as mandatory sequence nodes

---

## 11. BT Library Recommendation

**Recommended:** [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)

**Reasons:**
- ✅ C++ native (matches Autoware)
- ✅ ROS 2 integration available
- ✅ Visual editor (Groot2)
- ✅ XML-based tree definition
- ✅ Active development
- ✅ Industrial robotics proven

**Integration Path:**
```bash
# Add to Autoware workspace
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git

# Add to package dependencies
# Modify CMakeLists.txt and package.xml
```

---

## 12. References

### Autoware Documentation:
- Planning Design: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/
- Node Diagram: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/

### Code References:
- Scenario Selector: [autoware_scenario_selector/README.md](../src/universe/autoware_universe/planning/autoware_scenario_selector/README.md)
- Static Avoidance: [autoware_behavior_path_static_obstacle_avoidance_module/README.md](../src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/README.md)
- Run Out FSM: [state_machine.hpp](../src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_run_out_module/src/state_machine.hpp)

### Papers:
- Jerk Constrained Velocity Planning: Shimizu et al., ICRA 2022

---

## 13. Next Steps

1. **Design BT Architecture:**
   - Create detailed BT diagrams for each scenario
   - Define BT node interfaces (inputs/outputs)
   - Plan data flow between BT and Autoware modules

2. **Prototype Development:**
   - Start with simplest scenario (broken-down vehicle)
   - Implement basic BT nodes
   - Test in isolated environment

3. **CARLA Integration:**
   - Study Autoware-CARLA bridge
   - Prepare test scenarios
   - Setup simulation pipeline

4. **Baseline Metrics:**
   - Run current FSM system in CARLA
   - Record performance data
   - Establish comparison baseline

---

## Appendix A: FSM State Machine Examples Found

**Files with FSM/State Machine implementations:**

1. **Run Out Module:** `behavior_velocity_run_out_module/src/state_machine.{hpp,cpp}`
   - States: GO, STOP, APPROACH, UNKNOWN
   - 109 lines of state transition logic

2. **Crosswalk Module:** `behavior_velocity_crosswalk_module/include/.../util.hpp`
   - States for pedestrian interaction

3. **Intersection Module:** `behavior_velocity_intersection_module/src/scene_intersection.hpp`
   - Collision state machine

4. **Blind Spot Module:** `behavior_velocity_blind_spot_module/src/scene.hpp`
   - Turn signal and blind spot states

5. **No Stopping Area Module:** `behavior_velocity_no_stopping_area_module/src/scene_no_stopping_area.hpp`
   - No stopping area state management

All these modules are candidates for BT replacement or BT integration.

---

**End of Document**

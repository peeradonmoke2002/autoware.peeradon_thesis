# Stop Conditions Analysis - Before Crash Object

This document lists all conditions checked by Autoware before stopping the vehicle to avoid crashing with an obstacle.

## Overview

Autoware uses **22 conditions** organized into 6 categories to decide whether to stop for an object.

---

## 1. Object Detection Conditions (Must Pass ALL)

| # | Condition | Code Location | Description | Config Parameter |
|---|-----------|---------------|-------------|------------------|
| 1 | **Object exists** | `scene.cpp:204` | `!perception_objects.empty()` | - |
| 2 | **Within detection range** | `helper.cpp` | `50m < distance < 150m` | `min_forward_distance`, `max_forward_distance` |
| 3 | **Object is stopped** | `helper.cpp` | `velocity < 1.0 m/s for > 2.0s` | `th_moving_speed: 1.0`, `th_moving_time: 2.0` |
| 4 | **Object on ego lane** | `helper.cpp` | Object centerline overlaps with ego path | - |
| 5 | **Object blocking path** | `helper.cpp` | Lateral distance < vehicle width + margin | `lateral_margin.soft_margin: 0.5m` |
| 6 | **Not too close to centerline** | `helper.cpp` | `lateral_distance > min_threshold` | - |
| 7 | **Valid object type** | `parameters` | Car, truck, bus, trailer (not pedestrian, bicycle) | `target_object.car: true` |

**Code Reference:**
```cpp
// From static_obstacle_avoidance.param.yaml
target_object:
  car:
    th_moving_speed: 1.0  # m/s - objects slower = stopped
    th_moving_time: 2.0   # s - must be stopped this long
    lateral_margin:
      soft_margin: 0.5    # m
      hard_margin: 0.2    # m

detection_range:
  min_forward_distance: 50.0   # meters
  max_forward_distance: 150.0  # meters
```

---

## 2. Safety Validation Conditions

From `scene.cpp:112-120`:

```cpp
bool isExecutionReady() const {
    return avoid_data_.safe &&
           avoid_data_.comfortable &&
           avoid_data_.valid &&
           avoid_data_.ready;
}
```

| # | Flag | Meaning |
|---|------|---------|
| 8 | `safe` | Stop won't cause collision with other objects |
| 9 | `comfortable` | Deceleration is within comfort limits |
| 10 | `valid` | Path is geometrically valid |
| 11 | `ready` | System is ready to execute |

---

## 3. Ego Vehicle State Conditions

| # | Condition | Check | Data Source |
|---|-----------|-------|-------------|
| 12 | **Ego has valid pose** | Odometry data available | `/localization/kinematic_state` |
| 13 | **Ego on valid lane** | Within lanelet boundaries | `/map/vector_map` |
| 14 | **Ego velocity known** | Velocity data available | `/localization/kinematic_state` or `/vehicle/status/velocity_status` |

---

## 4. Stop Decision Logic

From `scene.cpp:590-604`:

```cpp
// Finds nearest stopped object that requires stopping
ObjectData stop_target_object = nullptr;

for (auto & o : target_objects) {
    // Condition 15: Check if avoidance is NOT possible
    if (!helper_->isAbsolutelyNotAvoidable(o)) {
        continue;  // Can avoid, don't stop
    }

    // Condition 16: Check if this is nearest object
    if (o.longitudinal < nearest_distance) {
        stop_target_object = o;
        nearest_distance = o.longitudinal;
    }
}

// Condition 17: Stop target found
if (stop_target_object != nullptr) {
    avoid_data_.stop_target_object = stop_target_object;
}
```

| # | Condition | Description |
|---|-----------|-------------|
| 15 | **Cannot avoid** | `isAbsolutelyNotAvoidable()` returns true |
| 16 | **Nearest object** | Closest object among all blocking objects |
| 17 | **Stop target assigned** | `stop_target_object != nullptr` |

**Key Check in `isExecutionRequested()` - `scene.cpp:91-110`:**

```cpp
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    // KEY DECISION POINT: Check if there's an object to stop for
    if (!!avoid_data_.stop_target_object) {  // ← Condition 17
        return true;
    }

    // ... other avoidance logic ...
}
```

---

## 5. Why Object is "Absolutely Not Avoidable"

From `helper.cpp - isAbsolutelyNotAvoidable()`:

Object must meet ONE of these reasons:

| # | Reason | Enum Value | Description |
|---|--------|------------|-------------|
| 18a | **Insufficient space** | `INSUFFICIENT_DRIVABLE_SPACE` | Not enough lateral room to go around object |
| 18b | **Need deceleration** | `NEED_DECELERATION` | Too close to object, must slow down/stop |
| 18c | **Ambiguous stopped vehicle** | `AMBIGUOUS_STOPPED_VEHICLE` | Uncertain if vehicle is parked or temporarily stopped |

**From `data_structs.hpp:42-69`:**
```cpp
enum class ObjectInfo {
    NONE = 0,
    // ignore reasons (can ignore these objects)
    OUT_OF_TARGET_AREA,
    MOVING_OBJECT,
    TOO_NEAR_TO_CENTERLINE,
    // unavoidable reasons (MUST stop for these)
    NEED_DECELERATION,            // 18b
    INSUFFICIENT_DRIVABLE_SPACE,  // 18a
    // others
    AMBIGUOUS_STOPPED_VEHICLE,    // 18c
};
```

---

## 6. Stop Point Calculation Conditions

| # | Condition | Description | Formula |
|---|-----------|-------------|---------|
| 19 | **Sufficient stop distance** | Distance to object > stopping distance at current speed | `stop_dist = v²/(2*a)` where a = max deceleration |
| 20 | **Stop point on path** | Stop point is within valid path boundaries | Must be on lanelet |
| 21 | **Stop point before object** | Stop point is before object, not past it | `stop_point.s < object.s` |

---

## Complete Decision Flow

```
Tick Start (10 Hz)
  ↓
┌─────────────────────────────────┐
│ 1. Object Detection (Cond 1-7) │
│ Is there a stopped object       │
│ blocking our path?              │
└─────────────────────────────────┘
  ↓ YES
┌─────────────────────────────────┐
│ 2. Safety Validation (Cond 8-11)│
│ Is it safe/comfortable/valid    │
│ to stop?                        │
└─────────────────────────────────┘
  ↓ YES
┌─────────────────────────────────┐
│ 3. Ego State Check (Cond 12-14) │
│ Do we know ego position/velocity│
└─────────────────────────────────┘
  ↓ YES
┌─────────────────────────────────┐
│ 4. Stop Decision (Cond 15-17)   │
│ Can we avoid?                   │
│   NO → Set as stop_target       │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ 5. Unavoidable Reason (Cond 18) │
│ Why can't avoid?                │
│ (space/distance/ambiguous)      │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ 6. Stop Point Calc (Cond 19-21) │
│ Where to stop?                  │
│ (distance/location/safety)      │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ 7. Execute Stop                 │
│ isExecutionRequested() = TRUE   │
│ → Publish stop path             │
└─────────────────────────────────┘
```

---

## Simplified for "Face Object and Stop" BT Implementation

For your thesis scope (broken-down vehicle only), you can use these **7 core conditions**:

### Minimum Required Conditions:

```python
def should_stop_for_object(obj, ego_state) -> bool:
    """
    Simplified stop decision for BT implementation.
    Returns True if vehicle should stop for this object.
    """

    # Must pass ALL these conditions:
    conditions = [
        obj.velocity < 1.0,                                    # 3. Object is stopped (< 1.0 m/s)
        obj.stopped_duration > 2.0,                            # 3. Stopped long enough (> 2.0s)
        is_on_ego_path(obj, ego_state),                        # 4. Object on our lane
        is_blocking_path(obj, ego_state),                      # 5. Object blocking path
        5.0 < distance_to_obj < 150.0,                         # 2. Within detection range
        obj.type in ['CAR', 'TRUCK', 'BUS'],                   # 7. Valid object type
        can_stop_safely(distance_to_obj, ego_state.velocity)   # 19. Can stop in time
    ]

    return all(conditions)
```

### Helper Functions Needed:

```python
def is_on_ego_path(obj, ego_state) -> bool:
    """Check if object is on ego's planned path (Condition 4)"""
    # Compare object position with ego's lane centerline
    # Return True if lateral distance < lane_width/2
    pass

def is_blocking_path(obj, ego_state) -> bool:
    """Check if object is blocking (Condition 5)"""
    lateral_distance = calculate_lateral_distance(obj, ego_state)
    vehicle_width = 1.8  # meters (typical)
    margin = 0.5  # meters (soft margin)

    return lateral_distance < (vehicle_width + margin)

def can_stop_safely(distance, ego_velocity) -> bool:
    """Check if we can stop before object (Condition 19)"""
    max_decel = 3.0  # m/s² (comfortable deceleration)
    safety_margin = 2.0  # meters (buffer distance)

    # Physics: stopping_distance = v² / (2 * a)
    stopping_distance = (ego_velocity ** 2) / (2 * max_decel)

    return distance > (stopping_distance + safety_margin)
```

---

## ROS Topics for BT Input

These are the topics your BT needs to subscribe to:

| Topic | Message Type | QoS | Contains |
|-------|-------------|-----|----------|
| `/perception/object_recognition/objects` | `PredictedObjects` | RELIABLE, depth=1 | All detected objects (Cond 1-7) |
| `/localization/kinematic_state` | `Odometry` | RELIABLE, depth=1 | Ego pose & velocity (Cond 12, 14) |
| `/map/vector_map` | `LaneletMapBin` | RELIABLE, depth=1, TRANSIENT_LOCAL | Map data (Cond 13) |
| `/planning/mission_planning/route` | `LaneletRoute` | RELIABLE, depth=1, TRANSIENT_LOCAL | Planned route (Cond 4) |

---

## What You Can Skip for Thesis

For "face object and stop" behavior (no avoidance), you can **skip**:

- ❌ Comfort checks (Condition 9) - just stop, don't optimize comfort
- ❌ Lane boundaries validation (Condition 13) - assume on lane
- ❌ Map data (skip `/map/vector_map`) - use simpler distance checks
- ❌ Route data (skip `/planning/mission_planning/route`) - assume straight path
- ❌ Ambiguous vehicle classification (Condition 18c) - assume all stopped = broken down

### Absolute Minimum (3 conditions):

For quick prototyping, you only need:

```python
def minimal_stop_decision(obj, ego_velocity) -> bool:
    """Absolute minimum for testing"""
    distance = calculate_distance(obj.position, ego_position)

    return (
        obj.velocity < 1.0 and          # Object stopped
        distance < 100.0 and            # Close enough
        distance > 10.0                 # Not too close
    )
```

---

## Summary Table

| Category | # Conditions | Required for Thesis? | Complexity |
|----------|--------------|---------------------|------------|
| **Object Detection** | 7 | ✅ YES (4-5 minimum) | Medium |
| **Safety Validation** | 4 | ⚠️ PARTIAL (safe only) | Low |
| **Ego State** | 3 | ✅ YES (2 minimum) | Low |
| **Stop Decision** | 3 | ✅ YES (all 3) | Low |
| **Unavoidable Reason** | 3 | ⚠️ PARTIAL (1 reason) | Medium |
| **Stop Point Calc** | 3 | ✅ YES (1 minimum) | Medium |
| **TOTAL** | **22** | **~10-12 recommended** | - |

---

## References

- **Main decision logic**: `/home/peeradon/autoware/src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`
  - Lines 91-110: `isExecutionRequested()`
  - Lines 112-120: `isExecutionReady()`
  - Lines 590-604: Stop target selection

- **Object classification**: `/home/peeradon/autoware/src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/include/autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp`
  - Lines 42-69: `ObjectInfo` enum

- **Configuration**: `/home/peeradon/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/.../static_obstacle_avoidance.param.yaml`

---

**Created**: 2025-10-02
**For**: Thesis - "Obstacle avoidance scenario selection in autonomous vehicles using behavior tree"
**Scope**: Broken-down vehicle detection and stop behavior

# Which Autoware Code to Copy for Your BT Implementation

Based on your requirement: **"if face object, ego should stop"**

This guide shows **exactly which code** from Autoware to copy and how to use it in your BT implementation.

---

## Key Files in Autoware Module

**Location:** `/home/peeradon/autoware/src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module`

**Key files:**
- `src/utils.cpp` - Object filtering logic
- `include/.../data_structs.hpp` - Data structures
- `include/.../type_alias.hpp` - ROS message types

---

## Code to Copy: Section by Section

### 1. **Data Structures** (Copy from data_structs.hpp)

#### A) Object Information Enum (Lines 42-69)

**What it does:** Classification reasons for objects (why ignore or can't avoid)

**Copy this:**
```cpp
enum class ObjectInfo {
  NONE = 0,
  // ignore reasons
  OUT_OF_TARGET_AREA,
  FURTHER_THAN_THRESHOLD,
  IS_NOT_TARGET_OBJECT,
  TOO_NEAR_TO_CENTERLINE,
  MOVING_OBJECT,          // ← Important! Objects that are moving
  // unavoidable reasons (need to stop!)
  NEED_DECELERATION,      // ← Stop required
  INSUFFICIENT_DRIVABLE_SPACE,  // ← Can't avoid, must stop
};
```

**Use in BT:** Condition nodes can return these reasons

---

#### B) ObjectData Structure (Lines 360-440)

**What it does:** Stores all info about detected object

**Key fields to copy:**
```cpp
struct ObjectData {
  PredictedObject object;  // ROS message from perception

  // Position info
  double to_centerline;    // Lateral distance from lane center
  double longitudinal;     // Longitudinal distance from ego
  double length;          // Object length

  // Movement tracking (KEY for stopped vehicle detection!)
  double move_time;       // How long has object been moving
  double stop_time;       // How long has object been stopped
  rclcpp::Time last_stop; // When did it last stop
  rclcpp::Time last_move; // When did it last move

  // Helper functions
  Pose getPose() const {
    return object.kinematics.initial_pose_with_covariance.pose;
  }

  Point getPosition() const {
    return object.kinematics.initial_pose_with_covariance.pose.position;
  }
};
```

**Use in BT:** Store this in blackboard, pass between nodes

---

### 2. **Velocity Calculation** (Copy from utils.cpp:1779-1781)

**What it does:** Calculate object velocity from ROS twist message

**Copy this function:**
```cpp
// From utils.cpp line 1779-1781
double getObjectVelocity(const PredictedObject & object)
{
  const auto & object_twist = object.kinematics.initial_twist_with_covariance.twist;
  const auto object_vel_norm = std::hypot(object_twist.linear.x, object_twist.linear.y);
  return object_vel_norm;
}
```

**Python equivalent:**
```python
def get_object_velocity(obj):
    vel = obj.kinematics.initial_twist_with_covariance.twist.linear
    return math.sqrt(vel.x**2 + vel.y**2)
```

**Use in BT:** ClassifyStoppedObjects node uses this

---

### 3. **Check if Object is Moving** (Copy from utils.cpp:244-250)

**What it does:** Determines if object has been moving long enough to be considered "moving" (not broken-down)

**Copy this function:**
```cpp
// From utils.cpp line 244-250
bool isMovingObject(
  const ObjectData & object,
  const double moving_time_threshold  // e.g., 2.0 seconds
)
{
  return object.move_time > moving_time_threshold;
}
```

**Logic:**
- If `move_time > 2.0s` → Object is moving (ignore it)
- If `move_time < 2.0s` → Object stopped recently (might be broken-down)

**Python equivalent:**
```python
def is_moving_object(obj, threshold=2.0):
    return obj.move_time > threshold
```

**Use in BT:** FilterObjects node - exclude moving vehicles

---

### 4. **Track Object Movement Over Time** (Copy from utils.cpp:1772-1825)

**What it does:** Updates `move_time` and `stop_time` by tracking object across frames

**This is the CORE logic for broken-down vehicle detection!**

**Copy this function:**
```cpp
// Simplified version from utils.cpp fillObjectMovingTime()
void updateObjectMovementTracking(
  ObjectData & object_data,
  std::vector<ObjectData> & stopped_objects_history,
  const double moving_speed_threshold  // e.g., 1.0 m/s
)
{
  // Get object velocity
  const auto & twist = object_data.object.kinematics.initial_twist_with_covariance.twist;
  const auto velocity = std::hypot(twist.linear.x, twist.linear.y);

  const auto is_stopped = velocity < moving_speed_threshold;
  const auto object_id = object_data.object.object_id;
  const auto now = rclcpp::Clock(RCL_ROS_TIME).now();

  // Find if we've seen this object before
  auto same_id_obj = std::find_if(
    stopped_objects_history.begin(),
    stopped_objects_history.end(),
    [&object_id](const auto & o) { return o.object.object_id == object_id; }
  );

  const bool is_new_object = (same_id_obj == stopped_objects_history.end());

  if (is_stopped) {
    // Object is currently stopped
    object_data.last_stop = now;
    object_data.move_time = 0.0;

    if (is_new_object) {
      // First time seeing this stopped object
      object_data.init_pose = object_data.getPose();
      object_data.stop_time = 0.0;
      object_data.last_move = now;
      stopped_objects_history.push_back(object_data);
    } else {
      // We've seen this object before - update stop duration
      same_id_obj->stop_time = (now - same_id_obj->last_move).seconds();
      same_id_obj->last_stop = now;
      same_id_obj->move_time = 0.0;

      object_data.stop_time = same_id_obj->stop_time;
      object_data.init_pose = same_id_obj->init_pose;
    }
  } else {
    // Object is currently moving
    if (is_new_object) {
      // First time seeing this moving object
      object_data.init_pose = object_data.getPose();
      object_data.move_time = std::numeric_limits<double>::infinity();
      object_data.stop_time = 0.0;
      object_data.last_move = now;
    } else {
      // Object was stopped before, now moving
      object_data.last_stop = same_id_obj->last_stop;
      object_data.move_time = (now - same_id_obj->last_stop).seconds();
      object_data.stop_time = 0.0;
      object_data.init_pose = object_data.getPose();

      // If moved long enough, remove from stopped history
      const double moving_time_threshold = 2.0;  // seconds
      if (object_data.move_time > moving_time_threshold) {
        stopped_objects_history.erase(same_id_obj);
      }
    }
  }
}
```

**Python equivalent:**
```python
def update_object_movement_tracking(obj_data, stopped_history, moving_speed_threshold=1.0):
    velocity = get_object_velocity(obj_data.object)
    is_stopped = velocity < moving_speed_threshold

    # Find if we've seen this object before
    same_id_obj = next((o for o in stopped_history if o.object.object_id == obj_data.object.object_id), None)
    is_new_object = same_id_obj is None
    current_time = time.time()

    if is_stopped:
        obj_data.last_stop = current_time
        obj_data.move_time = 0.0

        if is_new_object:
            obj_data.init_pose = obj_data.get_pose()
            obj_data.stop_time = 0.0
            obj_data.last_move = current_time
            stopped_history.append(obj_data)
        else:
            same_id_obj.stop_time = current_time - same_id_obj.last_move
            same_id_obj.last_stop = current_time
            same_id_obj.move_time = 0.0

            obj_data.stop_time = same_id_obj.stop_time
            obj_data.init_pose = same_id_obj.init_pose
    else:
        if is_new_object:
            obj_data.init_pose = obj_data.get_pose()
            obj_data.move_time = float('inf')
            obj_data.stop_time = 0.0
            obj_data.last_move = current_time
        else:
            obj_data.last_stop = same_id_obj.last_stop
            obj_data.move_time = current_time - same_id_obj.last_stop
            obj_data.stop_time = 0.0
            obj_data.init_pose = obj_data.get_pose()

            if obj_data.move_time > 2.0:  # moving_time_threshold
                stopped_history.remove(same_id_obj)
```

**Use in BT:** ClassifyStoppedObjects action node

---

### 5. **ROS Message Types** (Copy from type_alias.hpp)

**What it does:** Type definitions for ROS messages

**Copy these:**
```cpp
// From type_alias.hpp
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
```

**Use in BT:** Node input/output types

---

## Summary: Code You Need to Copy

### For Basic "Face Object → Stop" BT:

#### 1. **Data Structures**
```cpp
✓ ObjectData struct
✓ ObjectInfo enum
```

#### 2. **Utility Functions**
```cpp
✓ getObjectVelocity()          // Calculate velocity
✓ isMovingObject()             // Check if moving
✓ updateObjectMovementTracking() // Track over time
```

#### 3. **ROS Message Types**
```cpp
✓ PredictedObject
✓ PredictedObjects
✓ Pose, Point
```

---

## How to Use in Your BT

### BT Node Structure (Based on Your Diagram)

```
Root (Sequence)
├── DetectObjects (Condition)
│   └── Check if PredictedObjects exist
├── ClassifyObjects (Action)
│   └── Use updateObjectMovementTracking() to calculate stop_time
├── DecideAvoidOrStop (Fallback)
│   ├── AvoidBehavior (Sequence) - Future
│   └── StopBehavior (Sequence) ← IMPLEMENT THIS NOW
│       ├── DecideToStop (Condition)
│       │   └── Use isMovingObject() - if false (stopped) → SUCCESS
│       ├── SafetyCheck (Condition)
│       │   └── Check if safe to stop
│       └── ExecuteStop (Action)
│           └── Set velocity = 0.0
```

### Where Each Function Goes:

#### ClassifyObjects Node:
```cpp
class ClassifyObjects : public BT::SyncActionNode {
  BT::NodeStatus tick() override {
    // Get perception objects from blackboard
    auto perception_objects = getInput<PredictedObjects>("perception_objects");

    std::vector<ObjectData> classified_objects;

    for (const auto & obj : perception_objects.objects) {
      ObjectData obj_data;
      obj_data.object = obj;

      // ← USE updateObjectMovementTracking() HERE
      updateObjectMovementTracking(obj_data, stopped_objects_history_, 1.0);

      classified_objects.push_back(obj_data);
    }

    setOutput("classified_objects", classified_objects);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::vector<ObjectData> stopped_objects_history_;  // Track across ticks
};
```

#### DecideToStop Node:
```cpp
class DecideToStop : public BT::ConditionNode {
  BT::NodeStatus tick() override {
    // Get classified objects
    auto objects = getInput<std::vector<ObjectData>>("classified_objects");

    for (const auto & obj : objects.value()) {
      // ← USE isMovingObject() HERE
      if (!isMovingObject(obj, 2.0)) {  // 2.0s threshold
        // Object is stopped → need to stop!
        if (obj.stop_time >= 2.0) {  // Stopped for at least 2s
          std::cout << "Broken-down vehicle detected! Must stop!\n";
          setOutput("should_stop", true);
          return BT::NodeStatus::SUCCESS;  // Yes, should stop
        }
      }
    }

    setOutput("should_stop", false);
    return BT::NodeStatus::FAILURE;  // No need to stop
  }
};
```

#### ExecuteStop Node:
```cpp
class ExecuteStop : public BT::SyncActionNode {
  BT::NodeStatus tick() override {
    std::cout << "Executing STOP command\n";

    // Set ego velocity to 0
    double stop_velocity = 0.0;
    setOutput("target_velocity", stop_velocity);

    return BT::NodeStatus::SUCCESS;
  }
};
```

---

## Minimal Code to Start

**You don't need to copy everything!** Start with this minimal set:

### Step 1: Copy Velocity Function
```cpp
double getObjectVelocity(const PredictedObject & object) {
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  return std::hypot(twist.linear.x, twist.linear.y);
}
```

### Step 2: Copy Stop Check
```cpp
bool isStoppedObject(const PredictedObject & object, double threshold = 1.0) {
  return getObjectVelocity(object) < threshold;
}
```

### Step 3: Use in BT Node
```cpp
class DetectStoppedObject : public BT::ConditionNode {
  BT::NodeStatus tick() override {
    auto objects = getInput<PredictedObjects>("perception_objects");

    for (const auto & obj : objects.value().objects) {
      if (isStoppedObject(obj, 1.0)) {  // ← Use copied function
        std::cout << "Stopped object detected → STOP!\n";
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;  // No stopped objects
  }
};
```

---

## File Organization for Your Test

```
/home/peeradon/autoware/test/
├── autoware_utils.hpp           ← Copy utility functions here
│   ├── getObjectVelocity()
│   ├── isMovingObject()
│   └── updateObjectMovementTracking()
├── bt_stop_for_obstacle.cpp     ← Main BT implementation
│   ├── DetectObjects
│   ├── ClassifyObjects
│   ├── DecideToStop
│   └── ExecuteStop
└── CMakeLists.txt
```

---

## Next Steps

1. ✅ **Copy minimal functions first** (velocity check)
2. ✅ **Test with simple BT** (detect stopped → print stop)
3. ⏳ **Add movement tracking** (stop_time, move_time)
4. ⏳ **Add ROS 2 integration** (subscribe to perception topic)
5. ⏳ **Add path planning** (generate stop path)
6. ⏳ **Integrate with Autoware module**

---

## Key Takeaways

### What to Copy from Autoware:

| Feature | File | Lines | What to Copy |
|---------|------|-------|--------------|
| Velocity calculation | utils.cpp | 1779-1781 | `std::hypot(twist.linear.x, y)` |
| Is moving check | utils.cpp | 244-250 | `move_time > threshold` |
| Movement tracking | utils.cpp | 1772-1825 | Full function |
| ObjectData struct | data_structs.hpp | 360-440 | Struct definition |
| ROS types | type_alias.hpp | All | Using declarations |

### Don't Need to Copy:

- ❌ Path planning logic (too complex for now)
- ❌ Shift line generation (avoidance behavior - future)
- ❌ RTC approval system (not needed for simple test)
- ❌ Debug visualization (not essential)

---

**Ready to implement?** Check the next file: `bt_stop_for_obstacle.cpp` for the complete example!

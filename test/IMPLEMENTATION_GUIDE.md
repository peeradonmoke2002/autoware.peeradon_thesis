# Implementation Guide: BT for Obstacle Avoidance

## What You Have Now

Based on your requirement: **"If face object, ego should stop"**

I've created **2 BT examples** showing how to implement this using Autoware code.

---

## Files Created

### 1. Documentation

#### [AUTOWARE_CODE_TO_COPY.md](AUTOWARE_CODE_TO_COPY.md) ← **Read this first!**
- Shows **exactly which code** from Autoware to copy
- Line numbers and file locations
- Python equivalents for each C++ function
- Explains what each function does

####  [GETTING_STARTED.md](GETTING_STARTED.md)
- Your overall thesis roadmap
- C++ vs Python concepts
- Next steps guide

#### [README.md](README.md)
- Build and run instructions
- Technical reference

### 2. Code Examples

#### [simple_bt_obstacle_avoidance.cpp](simple_bt_obstacle_avoidance.cpp)
**Purpose:** Learn BT basics
- Detect → Filter → Classify flow
- Python-to-C++ explanations
- No Autoware code (simplified)
- **Start here to understand BT concepts**

#### [bt_stop_for_obstacle.cpp](bt_stop_for_obstacle.cpp) ← **This answers your question!**
**Purpose:** Implement "if face object → stop" with Autoware code
- Uses **real Autoware functions** for object tracking
- Implements **your BT diagram** flow
- Detects **broken-down vehicles** (stopped > 2s)
- Executes **stop command**
- **This is what you asked for!**

###3. Build Configuration

#### [CMakeLists.txt](CMakeLists.txt)
- Builds both examples
- Configured for BehaviorTree.CPP

---

## How the Code Answers Your Question

### Your Question:
> "I want to create case if face object the ego should stop. Based on flow BT diagram, can you suggest me what should line of code to copy and implement in my own version?"

### Answer:

## Step-by-Step: What Code to Copy

### From Autoware Module → Your BT

| What You Need | Autoware File | Lines | Copied To |
|---------------|---------------|-------|-----------|
| **Velocity calculation** | utils.cpp | 1780 | `getObjectVelocity()` in bt_stop_for_obstacle.cpp:73 |
| **Movement check** | utils.cpp | 244-250 | `isMovingObject()` in bt_stop_for_obstacle.cpp:85 |
| **Movement tracking** | utils.cpp | 1772-1825 | `updateObjectMovementTracking()` in bt_stop_for_obstacle.cpp:98 |
| **ObjectData struct** | data_structs.hpp | 360-440 | `struct ObjectData` in bt_stop_for_obstacle.cpp:41 |

---

## BT Implementation (Your Diagram → Code)

### Your BT Diagram Flow:

```
Root
├── Detect Objects
├── Classify Objects
└── Decide Avoid or Stop
    ├── Avoid Behavior (Future)
    └── Stop Behavior ← IMPLEMENTED!
        ├── Decide to Stop
        ├── Safety Check
        ├── Execute Stop
        └── Check Stop Complete
```

### Implemented in bt_stop_for_obstacle.cpp:

#### Node 1: DetectObjects (Line 179)
```cpp
class DetectObjects : public BT::ConditionNode {
    // Check if perception_objects exist
    // Return SUCCESS if objects detected
};
```
**Uses:** None (just checks if list is empty)

#### Node 2: ClassifyObjects (Line 205)
```cpp
class ClassifyObjects : public BT::SyncActionNode {
    BT::NodeStatus tick() override {
        // For each object:
        updateObjectMovementTracking(  // ← AUTOWARE FUNCTION!
            obj_data,
            stopped_objects_history_,
            MOVING_SPEED_THRESHOLD
        );
        // This tracks stop_time and move_time
    }

    ObjectDataArray stopped_objects_history_;  // Track across frames
};
```
**Uses:** `updateObjectMovementTracking()` from Autoware utils.cpp:1772-1825

#### Node 3: DecideToStop (Line 259)
```cpp
class DecideToStop : public BT::ConditionNode {
    BT::NodeStatus tick() override {
        for (const auto & obj : objects) {
            if (!isMovingObject(obj, 2.0)) {  // ← AUTOWARE FUNCTION!
                if (obj.stop_time >= 2.0) {
                    // BROKEN-DOWN VEHICLE!
                    return SUCCESS;  // Yes, should stop
                }
            }
        }
        return FAILURE;  // No need to stop
    }
};
```
**Uses:** `isMovingObject()` from Autoware utils.cpp:244-250

#### Node 4: SafetyCheck (Line 315)
```cpp
class SafetyCheck : public BT::ConditionNode {
    // Check if safe to stop
    // (Simplified in example)
};
```
**Uses:** None (simplified for now)

#### Node 5: ExecuteStop (Line 336)
```cpp
class ExecuteStop : public BT::SyncActionNode {
    BT::NodeStatus tick() override {
        std::cout << "STOPPING VEHICLE\n";
        double stop_velocity = 0.0;
        setOutput("target_velocity", stop_velocity);
        return SUCCESS;
    }
};
```
**Uses:** None (just sets target velocity to 0)

#### Node 6: CheckStopComplete (Line 359)
```cpp
class CheckStopComplete : public BT::ConditionNode {
    // Check if velocity < 0.1 m/s
    // Return SUCCESS if stopped
};
```
**Uses:** None (just checks velocity)

---

## How Movement Tracking Works (Key Concept!)

This is the **most important** part from Autoware that you copied:

### Problem:
How do you know if a stopped object is **broken-down** vs just **temporarily stopped** (e.g., at traffic light)?

### Autoware Solution (Copied in bt_stop_for_obstacle.cpp):

```cpp
void updateObjectMovementTracking(
    ObjectData & object_data,
    ObjectDataArray & stopped_objects_history,  // Track objects across frames!
    double moving_speed_threshold  // 1.0 m/s
)
{
    // Calculate velocity
    const auto velocity = object_data.getVelocity();
    const auto is_stopped = velocity < moving_speed_threshold;

    // Have we seen this object before?
    auto same_id_obj = find_if(..., [](o) {
        return o.object_id == object_data.object_id;
    });

    if (is_stopped) {
        if (is_new_object) {
            // First time seeing stopped object
            object_data.stop_time = 0.0;
            stopped_objects_history.push_back(object_data);
        } else {
            // Update how long it's been stopped
            object_data.stop_time = (now - last_move).seconds();
        }
    } else {
        // Object is moving
        if (was_stopped_before) {
            // Calculate how long it's been moving
            object_data.move_time = (now - last_stop).seconds();

            // If moved for > 2.0s, remove from stopped history
            if (object_data.move_time > 2.0) {
                stopped_objects_history.erase(same_id_obj);
            }
        }
    }
}
```

### Key Variables (From Autoware ObjectData struct):

```cpp
struct ObjectData {
    double stop_time;   // How long object has been stopped (seconds)
    double move_time;   // How long object has been moving (seconds)

    // If stop_time >= 2.0s → Broken-down vehicle!
    // If move_time > 2.0s → Remove from history (not broken-down)
};
```

---

## Building and Running

### Step 1: Install BehaviorTree.CPP

```bash
sudo apt update
sudo apt install libbehaviortree-cpp-dev
```

### Step 2: Build Examples

```bash
cd /home/peeradon/autoware/test
mkdir build && cd build
cmake ..
make
```

You should see:
```
-- BehaviorTree.CPP found: TRUE
-- Building BT examples:
--   - simple_bt_obstacle_avoidance (basic example)
--   - bt_stop_for_obstacle (with Autoware code)
[100%] Built target simple_bt_obstacle_avoidance
[100%] Built target bt_stop_for_obstacle
```

### Step 3: Run Examples

#### Example 1: Basic BT (Learn concepts)
```bash
./simple_bt_obstacle_avoidance
```

Expected output:
```
[DetectObjects] ✓ Detected 5 objects
[FilterObjects] ✓ Filtered to 3/5 objects
[ClassifyStoppedObjects] ✓ Classified 2 stopped objects
BT Status: SUCCESS ✓

Final Results:
- Stopped objects detected: 2
Broken-down vehicles to avoid:
  → obj_2 at 30m (velocity: 0.5 m/s, stopped: 3.5s)
  → obj_3 at 120m (velocity: 0.2 m/s, stopped: 5s)
```

#### Example 2: BT with Stop (Your requirement!)
```bash
./bt_stop_for_obstacle
```

Expected output:
```
Frame 1 (simulated perception update)
[DetectObjects] ✓ Detected 2 objects
[ClassifyObjects] Object car_moving_001 - vel: 10 m/s, stop_time: 0s
[ClassifyObjects] Object car_stopped_002 - vel: 0 m/s, stop_time: 0s
[DecideToStop] ✓ No stopped vehicles → Continue

Frame 2 (simulated perception update)
...stop_time: 1.0s...

Frame 3 (simulated perception update)
[ClassifyObjects] Object car_stopped_002 - vel: 0 m/s, stop_time: 2.1s
[DecideToStop] ⚠️  BROKEN-DOWN VEHICLE DETECTED!
  → Object ID: car_stopped_002
  → Velocity: 0 m/s
  → Stopped for: 2.1 seconds
  → Decision: MUST STOP!

[ExecuteStop] 🛑 STOPPING VEHICLE
  → Reason: Broken-down vehicle ahead
  → Setting target velocity = 0.0 m/s

✓ SUCCESS: BT correctly detected broken-down vehicle and stopped!
```

---

## Understanding the Output

### What Happens:

**Frame 1-2:** Object just stopped
- `stop_time < 2.0s`
- Decision: Continue (might be temporary stop)

**Frame 3+:** Object stopped for >= 2.0s
- `stop_time >= 2.0s`
- Decision: **STOP! This is a broken-down vehicle!**

This logic is **copied directly from Autoware**!

---

## Mapping to Your BT Diagram

```
Your Diagram                    |  Implementation File
--------------------------------|----------------------------------
Root                            |  XML tree definition (line 416)
├─ Detect Objects               |  DetectObjects class (line 179)
├─ Classify Objects             |  ClassifyObjects class (line 205)
└─ Decide Avoid or Stop (?)     |  Fallback node (line 435)
    ├─ Avoid Behavior (False)   |  AlwaysFailure (not implemented)
    └─ Stop Behavior →          |  Sequence (line 439)
        ├─ Decide to Stop       |  DecideToStop class (line 259)
        ├─ Safety Check         |  SafetyCheck class (line 315)
        ├─ Execute Stop         |  ExecuteStop class (line 336)
        └─ Check Stop Complete  |  CheckStopComplete class (line 359)
```

---

## Code Organization

```
bt_stop_for_obstacle.cpp:

Lines 1-70:   Data structures (PredictedObject, ObjectData)
Lines 73-96:  Autoware utility functions (copied!)
Lines 98-173: Object movement tracking (copied from Autoware!)
Lines 179-202: DetectObjects node
Lines 205-255: ClassifyObjects node ← Uses Autoware tracking!
Lines 259-312: DecideToStop node ← Uses Autoware is_moving check!
Lines 315-333: SafetyCheck node
Lines 336-356: ExecuteStop node
Lines 359-384: CheckStopComplete node
Lines 389-end: Main function (test scenario)
```

---

## What Code You Should Copy for Your Own Implementation

### Minimal Version (Start Here):

1. **Copy velocity calculation (3 lines)**
```cpp
// From bt_stop_for_obstacle.cpp line 73-77
double getObjectVelocity(const PredictedObject & object) {
    return std::hypot(object.twist.linear.x, object.twist.linear.y);
}
```

2. **Copy stop check (2 lines)**
```cpp
// From bt_stop_for_obstacle.cpp line 85-88
bool isMovingObject(const ObjectData & object, double threshold) {
    return object.move_time > threshold;
}
```

3. **Use in BT node**
```cpp
class YourStopNode : public BT::ConditionNode {
    BT::NodeStatus tick() override {
        auto objects = getInput<Objects>("objects");

        for (const auto & obj : objects) {
            if (getObjectVelocity(obj.object) < 1.0) {  // ← COPIED!
                std::cout << "Stopped object → STOP!\n";
                return SUCCESS;
            }
        }

        return FAILURE;
    }
};
```

### Full Version (For Thesis):

Copy the entire `updateObjectMovementTracking()` function to:
- Track objects across multiple frames
- Distinguish broken-down vehicles (stopped > 2s) from temporary stops
- Match Autoware's logic exactly

---

## Next Steps

### Phase 1: Test the Examples (NOW)
1. Build and run `bt_stop_for_obstacle`
2. Understand how movement tracking works
3. Modify thresholds (e.g., change `2.0s` to `3.0s`)
4. Add your own test scenarios

### Phase 2: Add ROS 2 Integration (NEXT)
1. Replace simulated `PredictedObjects` with ROS 2 subscriber
2. Subscribe to `/perception/object_recognition/objects`
3. Run with Autoware simulator

### Phase 3: Integrate with Autoware Module (LATER)
1. Copy this BT logic into `autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`
2. Replace FSM decision logic with BT
3. Keep Autoware's path planning utilities

---

## Summary

### Your Question:
"What lines of code to copy from Autoware to implement 'if face object → stop' in BT?"

### Answer:
✅ **File:** `bt_stop_for_obstacle.cpp`
✅ **Functions Copied:**
- `getObjectVelocity()` (line 73)
- `isMovingObject()` (line 85)
- `updateObjectMovementTracking()` (line 98)

✅ **Used in BT Nodes:**
- `ClassifyObjects` uses `updateObjectMovementTracking()`
- `DecideToStop` uses `isMovingObject()`

✅ **Result:**
When object is stopped for >= 2.0s:
- BT detects broken-down vehicle
- Executes stop command
- Target velocity = 0.0 m/s

**This is exactly what you asked for!** 🎯

---

## Questions?

1. **How does movement tracking work?**
   → See `updateObjectMovementTracking()` in bt_stop_for_obstacle.cpp:98

2. **Where is the stop decision made?**
   → `DecideToStop` node, line 259

3. **How do I modify the threshold?**
   → Change `STOP_TIME_THRESHOLD` in `DecideToStop` class

4. **Can I test without ROS?**
   → Yes! bt_stop_for_obstacle.cpp uses simulated data

5. **Is this the same logic as Autoware?**
   → Yes! Functions are copied from utils.cpp

---

Good luck with your thesis! 🚗🎓

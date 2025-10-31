# Quick Start: BT for "If Face Object → Stop"

## TL;DR

**You asked:** "Which code from Autoware to copy for 'if face object, ego should stop'?"

**Answer:** I've created [bt_stop_for_obstacle.cpp](bt_stop_for_obstacle.cpp) with Autoware code already copied!

---

## Files to Read

### 📖 Start Here (In Order):

1. **[IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)** ← **Read this first!**
   - Complete answer to your question
   - Shows which lines to copy from Autoware
   - Maps your BT diagram to code

2. **[AUTOWARE_CODE_TO_COPY.md](AUTOWARE_CODE_TO_COPY.md)**
   - Detailed breakdown of Autoware functions
   - Line numbers and explanations
   - Python equivalents

3. **[bt_stop_for_obstacle.cpp](bt_stop_for_obstacle.cpp)**
   - **THE CODE YOU NEED!**
   - Implements your BT diagram
   - Uses Autoware movement tracking

---

## Build and Run (3 Commands)

```bash
cd /home/peeradon/autoware/test
mkdir build && cd build
cmake .. && make

# Run the example
./bt_stop_for_obstacle
```

**Expected:** BT detects broken-down vehicle after 2-3 frames and executes stop command

---

## What Code Was Copied from Autoware?

### Function 1: Get Velocity
**From:** `utils.cpp` line 1780
**To:** `bt_stop_for_obstacle.cpp` line 73

```cpp
double getObjectVelocity(const PredictedObject & object) {
    return std::hypot(object.twist.linear.x, object.twist.linear.y);
}
```

### Function 2: Check if Moving
**From:** `utils.cpp` line 244-250
**To:** `bt_stop_for_obstacle.cpp` line 85

```cpp
bool isMovingObject(const ObjectData & object, double threshold) {
    return object.move_time > threshold;
}
```

### Function 3: Track Movement (Most Important!)
**From:** `utils.cpp` line 1772-1825
**To:** `bt_stop_for_obstacle.cpp` line 98-173

```cpp
void updateObjectMovementTracking(
    ObjectData & object_data,
    ObjectDataArray & stopped_objects_history,
    double moving_speed_threshold
) {
    // Tracks how long object has been stopped
    // If stopped >= 2.0s → broken-down vehicle!
}
```

---

## BT Nodes (Your Diagram → Code)

```
Your BT Diagram              Code Location
─────────────────────────────────────────────────────
DetectObjects              → line 179
ClassifyObjects            → line 205 (uses Autoware tracking!)
DecideToStop (?)           → line 259 (uses is_moving check!)
├─ SafetyCheck             → line 315
├─ ExecuteStop             → line 336
└─ CheckStopComplete       → line 359
```

---

## Key Concept: Movement Tracking

### Problem:
How to distinguish:
- Broken-down vehicle (stopped for a long time)
- Temporary stop (e.g., at traffic light)

### Autoware Solution (Now Copied in bt_stop_for_obstacle.cpp):

```cpp
struct ObjectData {
    double stop_time;  // How long stopped
    double move_time;  // How long moving

    // If stop_time >= 2.0s → Broken-down!
};

// Update every frame:
updateObjectMovementTracking(obj, history, 1.0);

// Decision:
if (!isMovingObject(obj, 2.0)) {
    if (obj.stop_time >= 2.0) {
        STOP!  // Broken-down vehicle
    }
}
```

---

## Example Output

```
Frame 1:
[ClassifyObjects] car_stopped_002 - stop_time: 0.5s
[DecideToStop] ✓ No broken-down vehicles → Continue

Frame 2:
[ClassifyObjects] car_stopped_002 - stop_time: 1.5s
[DecideToStop] ✓ No broken-down vehicles → Continue

Frame 3:
[ClassifyObjects] car_stopped_002 - stop_time: 2.1s
[DecideToStop] ⚠️  BROKEN-DOWN VEHICLE DETECTED!
  → Stopped for: 2.1 seconds
  → Decision: MUST STOP!

[ExecuteStop] 🛑 STOPPING VEHICLE
  → Setting target velocity = 0.0 m/s

✓ SUCCESS!
```

---

## For Your Thesis

### What You Have Now:
✅ BT example with Autoware code
✅ Object movement tracking
✅ Stop decision logic
✅ Broken-down vehicle detection

### Next Steps:
1. ⏳ Add ROS 2 integration (subscribe to perception)
2. ⏳ Test with Autoware simulator
3. ⏳ Integrate into Autoware module
4. ⏳ Compare FSM vs BT

---

## Files Created for You

```
/home/peeradon/autoware/test/
├── IMPLEMENTATION_GUIDE.md           ← Complete guide
├── AUTOWARE_CODE_TO_COPY.md          ← Which code to copy
├── QUICK_START.md                    ← This file
├── bt_stop_for_obstacle.cpp          ← THE CODE YOU NEED
├── simple_bt_obstacle_avoidance.cpp  ← Learn BT basics
├── CMakeLists.txt                    ← Build config
└── README.md                         ← Technical docs
```

---

## Summary

**Your Question:**
> "Which lines of code from Autoware should I copy to implement 'if face object → stop' in BT?"

**Answer:**
✅ **3 functions copied** from `utils.cpp`:
- `getObjectVelocity()` (line 1780)
- `isMovingObject()` (line 244-250)
- `updateObjectMovementTracking()` (line 1772-1825)

✅ **Implemented in:** [bt_stop_for_obstacle.cpp](bt_stop_for_obstacle.cpp)

✅ **BT flow:** Detect → Classify → Decide → Stop

✅ **Works:** Detects broken-down vehicles (stopped >= 2s) and stops

**Build and run it now!** 🚀

# Planning Execution Cycle: How Often Does Autoware Check for Avoidable Objects?

**Your Question:** In `behavior_path_planner`, **how often** does it check "are there avoidable objects?" and **what triggers** this check?

**Answer:** The check happens **continuously at 10 Hz** (every 100ms) in a timer loop. Let me show you the complete cycle.

---

## 1. The Main Timer Loop (10 Hz)

### Timer Setup

**Location:** `behavior_path_planner_node.cpp:123-129`

```cpp
// Start timer
{
    const auto planning_hz = declare_parameter<double>("planning_hz");  // Default: 10.0 Hz
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
        this, get_clock(), period_ns,
        std::bind(&BehaviorPathPlannerNode::run, this)  // ← Call run() every 100ms
    );
}
```

**Configuration:**
- **Frequency:** 10 Hz (configurable via parameter)
- **Period:** 100 milliseconds
- **Callback:** `BehaviorPathPlannerNode::run()`

This means **every 100ms**, the system executes the entire planning cycle!

---

## 2. Complete Execution Cycle (Every 100ms)

### Step-by-Step Flow

```
Timer triggers (every 100ms)
        ↓
┌─────────────────────────────────────────────────────────────┐
│ BehaviorPathPlannerNode::run()                              │
│ Location: behavior_path_planner_node.cpp:300                │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ STEP 1: takeData() - Update latest sensor data             │
│ Location: behavior_path_planner_node.cpp:159-251            │
│                                                             │
│ Updates planner_data_ with:                                 │
│ - dynamic_object (from /perception/objects)                 │
│ - self_odometry (from /localization/kinematic_state)        │
│ - occupancy_grid (from /perception/occupancy_grid)          │
│ - traffic_signals (from /perception/traffic_signals)        │
│ - map, route, acceleration, etc.                            │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ STEP 2: isDataReady() - Check all mandatory data exists    │
│ Location: behavior_path_planner_node.cpp:254-298            │
│                                                             │
│ Checks:                                                     │
│ - ✓ dynamic_object available?                               │
│ - ✓ self_odometry available?                                │
│ - ✓ map available?                                          │
│ - ✓ route available?                                        │
│                                                             │
│ If any missing → return (skip this cycle)                   │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ STEP 3: Check scenario                                     │
│ Location: behavior_path_planner_node.cpp:313-315            │
│                                                             │
│ if (scenario != LANEDRIVING) return;                        │
│                                                             │
│ Behavior path planner ONLY runs in LANEDRIVING scenario     │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ STEP 4: planner_manager_->run(planner_data_)                │
│ Location: behavior_path_planner_node.cpp:372                │
│                                                             │
│ THIS is where ALL modules are executed!                     │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ Inside PlannerManager::run()                                │
│ Location: planner_manager.cpp:120                           │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
        ┌──────────┴──────────┐
        │                     │
        ↓                     ↓
   For each SLOT       For each MODULE in slot
   (Slot 1-4)         (static_obstacle_avoidance, etc.)
        │                     │
        └──────────┬──────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ For StaticObstacleAvoidanceModule:                          │
│                                                             │
│ A) module->updateData()  ← Update module's internal state  │
│ B) module->isExecutionRequested()  ← Check if needed       │
│ C) module->isExecutionReady()  ← Check if safe             │
│ D) module->plan()  ← Generate avoidance path               │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ STEP 5: Publish result                                     │
│ Location: behavior_path_planner_node.cpp:374-379            │
│                                                             │
│ path_publisher_->publish(path);                             │
│ turn_signal_publisher_->publish(turn_signal);               │
└─────────────────────────────────────────────────────────────┘
                   ↓
    Wait 100ms, then repeat!
```

---

## 3. When Object Check Actually Happens

### Inside `StaticObstacleAvoidanceModule::updateData()`

**Location:** `scene.cpp:1518-1567`

**This function is called EVERY cycle (10 Hz) when the module is loaded:**

```cpp
void StaticObstacleAvoidanceModule::updateData()
{
    // 1. Initialize helper with planner data
    helper_->setData(planner_data_);  // ← Gets latest object data

    // 2. Reset debug data
    debug_data_ = DebugData();
    avoid_data_.update();

    // 3. Fill fundamental data (including object detection)
    fillFundamentalData(avoid_data_, debug_data_);
    //    ↑
    //    └─ This calls fillAvoidanceTargetObjects()
    //       which checks ALL objects from perception

    // 4. Generate shift lines (avoidance paths)
    fillShiftLine(avoid_data_, debug_data_);

    // 5. Update ego status (FSM state)
    fillEgoStatus(avoid_data_, debug_data_);

    // 6. Update debug visualization
    fillDebugData(avoid_data_, debug_data_);
}
```

### Inside `fillFundamentalData()` → `fillAvoidanceTargetObjects()`

**Location:** `scene.cpp:350`

```cpp
void StaticObstacleAvoidanceModule::fillAvoidanceTargetObjects(
    AvoidancePlanningData & data, DebugData & debug) const
{
    // Get ALL detected objects from perception
    const auto & objects = planner_data_->dynamic_object;  // ← Latest perception data

    // Loop through EVERY object
    for (const auto & object : objects->objects) {

        // === OBJECT CLASSIFICATION LOGIC (runs for EVERY object) ===

        // 1. Type check
        if (!isTargetObjectType(object)) {
            debug.filtered_objects.push_back({object, "NOT_TARGET_TYPE"});
            continue;  // Skip this object
        }

        // 2. Distance check
        const auto distance = calcDistance(ego_pose, object.pose);
        if (distance > parameters_->max_forward_distance) {
            debug.filtered_objects.push_back({object, "TOO_FAR"});
            continue;  // Skip this object
        }

        // 3. Movement check (velocity tracking)
        const auto velocity = object.kinematics.twist.linear.x;
        if (velocity > parameters_->th_moving_speed) {
            debug.filtered_objects.push_back({object, "MOVING_OBJECT"});
            continue;  // Skip this object
        }

        // Check if stopped long enough
        const auto stopped_time = getStoppedTime(object);
        if (stopped_time < parameters_->th_moving_time) {
            debug.filtered_objects.push_back({object, "NOT_STOPPED_LONG_ENOUGH"});
            continue;  // Skip this object
        }

        // 4. Position check (on road edge?)
        const auto offset = calcOffsetFromCenterline(object);
        if (offset < parameters_->th_offset_from_centerline) {
            debug.filtered_objects.push_back({object, "TOO_NEAR_CENTERLINE"});
            continue;  // Skip this object
        }

        // 5. Feasibility check
        if (!canAvoid(object)) {
            debug.filtered_objects.push_back({object, "CANNOT_AVOID"});
            continue;  // Skip this object
        }

        // === OBJECT PASSED ALL CHECKS → ADD TO AVOIDANCE TARGETS ===
        data.target_objects.push_back(object);
    }

    // Return: data.target_objects now contains ALL avoidable objects
}
```

---

## 4. Trigger Flow Diagram

```
Time: t=0ms
        ↓
┌─────────────────────────────────────┐
│ Perception publishes objects        │
│ Topic: /perception/objects          │
│ Frequency: ~10 Hz                   │
└──────────────┬──────────────────────┘
               ↓
        (ROS message queue)
               ↓
Time: t=100ms (timer triggers)
        ↓
┌─────────────────────────────────────┐
│ BehaviorPathPlannerNode::run()      │
│                                     │
│ 1. takeData()                       │
│    ├─ Read latest objects           │
│    └─ Store in planner_data_        │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ planner_manager_->run()             │
│                                     │
│ For each module:                    │
│   ├─ updateData() ← HERE!           │
│   │   └─ fillAvoidanceTargetObjects()│
│   │       └─ Check EVERY object     │
│   │           ├─ Type check          │
│   │           ├─ Velocity check      │
│   │           ├─ Position check      │
│   │           └─ Classify as avoidable│
│   │                                  │
│   ├─ isExecutionRequested()         │
│   │   └─ Return true if targets exist│
│   │                                  │
│   ├─ isExecutionReady()             │
│   │   └─ Return true if safe        │
│   │                                  │
│   └─ plan()                         │
│       └─ Generate avoidance path    │
└──────────────┬──────────────────────┘
               ↓
Time: t=200ms (next timer cycle)
        ↓
    (Repeat entire process)
```

---

## 5. Answer to Your Question

### Q: How often does it check for avoidable objects?

**A: Every 100ms (10 Hz)**

The check happens in this sequence:

1. **Timer triggers** (every 100ms)
2. **takeData()** - Get latest objects from perception
3. **planner_manager_->run()** - Execute all modules
4. **For static_obstacle_avoidance module:**
   - `updateData()` is called
   - Inside: `fillAvoidanceTargetObjects()` checks EVERY object
   - Applies classification rules to each object
   - Updates `avoid_data_.target_objects` list
5. **isExecutionRequested()** checks if `target_objects` is not empty
6. If yes → module runs, if no → module skips

### Q: What triggers the check?

**A: ROS 2 Timer (10 Hz)**

The trigger is **time-based**, NOT event-based:
- ✅ Runs every 100ms regardless of object changes
- ✅ Always processes latest perception data
- ❌ NOT triggered by "new object detected"
- ❌ NOT triggered by "object moved"

### Q: What if new objects appear between cycles?

**A: They are detected in the next cycle**

Example timeline:
```
t=0ms:   Timer triggers → Check objects (0 found)
t=50ms:  New broken-down car appears (perception detects it)
t=100ms: Timer triggers → Check objects (1 found!) → Start avoidance
t=200ms: Timer triggers → Check objects (still 1) → Continue avoidance
```

Maximum detection delay: **100ms** (one planning cycle)

---

## 6. Data Flow Diagram

```
┌────────────────────┐
│  Perception Node   │  (Runs at ~10 Hz)
│  - Detects objects │
│  - Classifies type │
└──────┬─────────────┘
       │ Publishes to /perception/objects
       ↓
┌────────────────────┐
│  ROS 2 Topic       │
│  (Message Queue)   │
└──────┬─────────────┘
       │
       ↓ (Read in takeData())
┌────────────────────────────────────────┐
│  planner_data_->dynamic_object         │
│  (Shared data structure)               │
└──────┬─────────────────────────────────┘
       │
       ↓ (Accessed in updateData())
┌────────────────────────────────────────┐
│  StaticObstacleAvoidanceModule         │
│                                        │
│  fillAvoidanceTargetObjects():         │
│  for (object : dynamic_object) {       │
│    if (isTargetType(object) &&         │
│        isStopped(object) &&            │
│        isOnRoadEdge(object)) {         │
│      target_objects.push_back(object); │
│    }                                   │
│  }                                     │
└──────┬─────────────────────────────────┘
       │
       ↓
┌────────────────────────────────────────┐
│  avoid_data_.target_objects            │
│  (List of avoidable objects)           │
└──────┬─────────────────────────────────┘
       │
       ↓ (Used in decision)
┌────────────────────────────────────────┐
│  isExecutionRequested()                │
│  return !target_objects.empty()        │
└────────────────────────────────────────┘
```

---

## 7. Important Timing Considerations

### Synchronization

**Perception → Planning:**
- Perception publishes at ~10 Hz
- Planning consumes at 10 Hz
- They are **NOT perfectly synchronized**
- Planning uses **latest available** data

**Worst Case Latency:**
```
Perception detects object at t=0
Planning cycle just finished at t=1ms
Next planning cycle: t=100ms
→ Latency: ~99ms
```

**Best Case Latency:**
```
Perception detects object at t=99ms
Planning cycle starts at t=100ms
→ Latency: ~1ms
```

### Object Tracking Over Time

The module tracks object history to determine "stopped duration":

```cpp
// In isObjectStopped():
// Need to track velocity over time to confirm "stopped for 2.0 seconds"

t=0s:   Object velocity = 0.5 m/s → Mark as "potentially stopped"
t=0.1s: Object velocity = 0.3 m/s → Still potentially stopped
t=0.2s: Object velocity = 0.2 m/s → Still potentially stopped
...
t=2.0s: Object velocity < 1.0 m/s for 2.0s → CONFIRMED STOPPED
        → Add to target_objects
```

---

## 8. Configuration Parameters

**Planning Frequency:**
```yaml
# File: behavior_path_planner.param.yaml
planning_hz: 10.0  # Can be changed (e.g., to 20.0 for faster response)
```

**Object Detection Thresholds:**
```yaml
# File: static_obstacle_avoidance.param.yaml
target_object:
  car:
    th_moving_speed: 1.0      # m/s - velocity threshold
    th_moving_time: 2.0       # s - stopped duration threshold
```

---

## 9. Summary

| Question | Answer |
|----------|--------|
| **How often?** | Every 100ms (10 Hz) |
| **What triggers?** | ROS 2 Timer (time-based) |
| **When are objects checked?** | In `updateData()` → `fillAvoidanceTargetObjects()` |
| **Every object checked?** | Yes, loops through ALL perception objects |
| **Detection delay?** | Max 100ms (one cycle) |
| **Continuous or event-based?** | Continuous (every cycle) |

**Key Insight for Your BT Implementation:**

The BT will also be called at **10 Hz**. Your BT condition nodes (like `IsObjectAvoidable?`) will be evaluated every 100ms, just like the current FSM checks. This is **reactive enough** for obstacle avoidance since:
- Human reaction time: ~250ms
- Vehicle at 50 km/h travels 1.4m in 100ms
- Object classification needs 2 seconds of observation anyway

---

**End of Document**
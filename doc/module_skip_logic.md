# Module Skip Logic: What Happens When No Avoidable Objects?

**Your Question:** If no avoidable objects, does it skip this module and do the next slot?

**Answer:** YES! Exactly right. Let me show you the exact code flow.

---

## 1. The Skip Logic

### Code Location: `planner_manager.cpp:560-562`

```cpp
manager_ptr->updateIdleModuleInstance();
if (!manager_ptr->isExecutionRequested(previous_module_output)) {
    continue;  // ← SKIP THIS MODULE, go to next module in slot
}
```

**Translation:**
- If `isExecutionRequested()` returns `false` (no avoidable objects)
- Then `continue` → Skip to next module in the loop
- Module does NOT execute, does NOT generate path

---

## 2. Complete Execution Flow

### Every 100ms, For Each Slot:

```cpp
// Pseudo-code of actual execution

for (Slot slot : [Slot1, Slot2, Slot3, Slot4]) {

    for (Module module : slot.modules) {

        // DECISION POINT 1: Should this module run?
        if (!module->isExecutionRequested()) {
            continue;  // ← SKIP to next module
        }

        // DECISION POINT 2: Is module ready to run?
        if (!module->isExecutionReady()) {
            // Module wants to run but not safe yet
            // Use fallback or wait
            continue;  // ← SKIP to next module
        }

        // EXECUTE MODULE
        auto path = module->plan();
        merged_path = merge(merged_path, path);
    }

    // After all modules in this slot processed...
    if (merged_path.valid) {
        break;  // Use this path, don't process remaining slots
    }
}
```

---

## 3. Real Example with Static Obstacle Avoidance

### Scenario: No Broken-Down Vehicles

```
Timer triggers (t=100ms)
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 1: start_planner                                   │
│ - isExecutionRequested() → false (not at start)         │
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 2: Multiple modules checked in order               │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: side_shift                                      │
│ - isExecutionRequested() → false (no shift requested)   │
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: static_obstacle_avoidance                       │
│                                                         │
│ 1. updateData()                                         │
│    └─ fillAvoidanceTargetObjects()                      │
│       └─ Check all objects... found 0 avoidable        │
│                                                         │
│ 2. isExecutionRequested()                               │
│    └─ target_objects.empty() == true                    │
│    └─ return false ← NO EXECUTION NEEDED                │
│                                                         │
│ Result: SKIP ❌                                          │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: lane_change_left                                │
│ - isExecutionRequested() → false (no lane change needed)│
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: lane_change_right                               │
│ - isExecutionRequested() → false (no lane change needed)│
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: bidirectional_traffic                           │
│ - isExecutionRequested() → false (not in narrow road)   │
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 2 Result: No modules executed                      │
│ → Use reference path (no modifications)                 │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 3: goal_planner                                    │
│ - isExecutionRequested() → false (not near goal)        │
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 4: dynamic_obstacle_avoidance                      │
│ - Usually disabled by default                           │
│ - SKIP ❌                                                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Final Result:                                           │
│ - Use reference path (lane following)                   │
│ - No avoidance maneuvers                                │
│ - Publish path                                          │
└─────────────────────────────────────────────────────────┘
```

---

### Scenario: Broken-Down Vehicle Detected!

```
Timer triggers (t=100ms)
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 1: start_planner                                   │
│ - SKIP ❌ (same as before)                               │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 2: Multiple modules...                             │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: static_obstacle_avoidance                       │
│                                                         │
│ 1. updateData()                                         │
│    └─ fillAvoidanceTargetObjects()                      │
│       └─ Found 1 stopped car on road edge! ✓            │
│                                                         │
│ 2. isExecutionRequested()                               │
│    └─ target_objects.size() == 1                        │
│    └─ return true ← YES, EXECUTE ME!                    │
│                                                         │
│ 3. isExecutionReady()                                   │
│    └─ Check safety... path is safe ✓                    │
│    └─ return true ← YES, READY!                         │
│                                                         │
│ 4. plan()                                               │
│    └─ Generate lateral shift path                       │
│    └─ Output: path with avoidance                       │
│                                                         │
│ Result: EXECUTE ✅ Generate avoidance path              │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Module: lane_change_left                                │
│ - Can execute simultaneously if needed                  │
│ - isExecutionRequested() → depends on situation         │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Slot 2 Result: Avoidance path generated                 │
│ → Use modified path (with lateral shift)                │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Remaining slots still checked but may be skipped        │
└─────────────────────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────────────────────┐
│ Final Result:                                           │
│ - Publish avoidance path                                │
│ - Vehicle will shift laterally around obstacle          │
└─────────────────────────────────────────────────────────┘
```

---

## 4. The `isExecutionRequested()` Implementation

### For Static Obstacle Avoidance

**Location:** `scene.cpp:91-110`

```cpp
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    // Check if there's a stop target
    if (!!avoid_data_.stop_target_object) {
        return true;  // ← Must execute (urgent stop needed)
    }

    // Check if there's any shift line (avoidance path)
    if (avoid_data_.new_shift_line.empty()) {
        return false;  // ← NO execution needed (no path to execute)
    }

    // Check if there are avoidable objects
    return std::any_of(
        avoid_data_.target_objects.begin(),
        avoid_data_.target_objects.end(),
        [this](const auto & o) {
            return !helper_->isAbsolutelyNotAvoidable(o);
        }
    );
    // ↑ Returns true if ANY object is avoidable
    // ↑ Returns false if target_objects is empty or all unavoidable
}
```

**What this means:**

| Condition | Return Value | Module Action |
|-----------|-------------|---------------|
| No objects detected | `false` | SKIP ❌ |
| Objects detected but all moving | `false` | SKIP ❌ |
| Objects detected but not on road edge | `false` | SKIP ❌ |
| Objects detected, stopped, on road edge | `true` | EXECUTE ✅ |

---

## 5. Performance Impact

### Why Skip Instead of Always Running?

**Computational Efficiency:**
- If module runs but does nothing → Wasted CPU cycles
- Planning system has tight real-time constraints (100ms cycle)
- Skipping saves ~10-20ms per unnecessary module

**Example Timing:**

```
All Modules Always Run:
├─ start_planner: 5ms (unnecessary)
├─ static_obstacle_avoidance: 15ms (unnecessary if no objects)
├─ lane_change: 10ms (unnecessary)
├─ goal_planner: 8ms (unnecessary)
└─ Total: 38ms wasted

With Skip Logic:
├─ Check isExecutionRequested(): 0.5ms × 4 = 2ms
├─ Only run necessary modules
└─ Total: 2-5ms (saves 30-35ms!)
```

**This allows:**
- Faster planning cycles
- More modules can be added
- System can run at higher frequency if needed

---

## 6. Important Edge Cases

### Case 1: Module Was Running, Now No Objects

```
t=0ms:   Object detected → module RUNNING
t=100ms: Still avoiding → module RUNNING
t=200ms: Object gone! → isExecutionRequested() returns... true or false?
```

**Answer:** Depends on implementation!

```cpp
// In isExecutionRequested():
if (avoid_data_.new_shift_line.empty()) {
    return false;  // ← Will skip
}
```

**BUT** if ego is still on the shift line:
- Shift line exists
- Module continues to run (returns to center)
- Only stops when fully returned to original lane

### Case 2: Object Appears Mid-Cycle

```
t=0ms:   Planning cycle starts, no objects → static_obstacle_avoidance SKIPS
t=50ms:  New object appears (perception detects it)
t=100ms: Next planning cycle → object IS detected → module EXECUTES
```

Maximum detection delay: 100ms (one cycle)

### Case 3: Multiple Modules Want to Run

```
Slot 2 modules:
├─ static_obstacle_avoidance: isExecutionRequested() = true
├─ lane_change_left: isExecutionRequested() = true
└─ Both can run? YES! (simultaneous execution allowed)

Configuration:
static_obstacle_avoidance:
  enable_simultaneous_execution_as_approved_module: true  ← Can run with others

lane_change_left:
  enable_simultaneous_execution_as_approved_module: true  ← Can run with others
```

Paths are merged:
1. Avoidance shifts path laterally
2. Lane change modifies on top of that
3. Final path has both behaviors

---

## 7. Summary Table

| Situation | isExecutionRequested() | Module Action | Next Step |
|-----------|------------------------|---------------|-----------|
| No objects in perception | `false` | SKIP | Check next module in slot |
| Objects detected but moving | `false` | SKIP | Check next module in slot |
| Objects stopped < 2.0s | `false` | SKIP | Check next module in slot |
| Objects stopped but centered | `false` | SKIP | Check next module in slot |
| **Objects stopped, on edge** | `true` | Check safety | If safe → EXECUTE, else SKIP |
| Already avoiding (shift line active) | `true` | Continue | Keep executing until done |

---

## 8. Code Reference for Your BT Implementation

When you implement BT, the skip logic stays the same:

```python
# Current FSM approach
def isExecutionRequested(self):
    if no_avoidable_objects:
        return False  # ← Skip
    return True

# Your BT approach
def isExecutionRequested(self):
    # Tick BT root node
    bt_result = behavior_tree.tick()

    if bt_result == BT.FAILURE:  # No avoidable objects
        return False  # ← Skip
    return True  # BT wants to run
```

**The core (PlannerManager) doesn't change!**
- Still calls `isExecutionRequested()`
- Still skips if returns `false`
- Still executes if returns `true`

---

## 9. Visual Summary

```
┌──────────────────────────────────────────────────────┐
│  Planning Cycle (Every 100ms)                        │
│                                                      │
│  For each module in slot:                            │
│                                                      │
│    ┌────────────────────────────────────┐           │
│    │ isExecutionRequested()?            │           │
│    └────────┬──────────────────┬────────┘           │
│             │                  │                     │
│        false│                  │true                 │
│     ┌───────▼─────┐       ┌───▼────────────────┐    │
│     │ SKIP ❌     │       │ isExecutionReady()?│    │
│     │ continue;   │       └───┬────────────┬───┘    │
│     └─────────────┘           │            │         │
│                          false│            │true     │
│                      ┌────────▼───┐   ┌───▼─────┐   │
│                      │ SKIP ❌    │   │ EXECUTE │   │
│                      │ continue;  │   │ plan()  │   │
│                      └────────────┘   └─────────┘   │
│                                                      │
│  Next module in slot or next slot                   │
└──────────────────────────────────────────────────────┘
```

**Yes, if no avoidable objects:**
- ✅ Module is skipped (`continue`)
- ✅ Next module in same slot is checked
- ✅ If all modules in slot skip → use reference path
- ✅ Move to next slot

**This is exactly what you understood! 🎯**

---

**End of Document**
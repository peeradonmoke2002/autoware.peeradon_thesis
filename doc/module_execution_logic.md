# Module Execution Logic: How Core Decides to Call Static Obstacle Avoidance

**Your Question:** Does the core just blindly call `StaticObstacleAvoidanceModuleManager`, or is there logic to decide WHEN to call it?

**Answer:** There IS decision logic! The module itself implements **decision functions** that the core calls to determine if/when the module should run.

---

## Key Concept: Module Self-Decides, Core Orchestrates

The **PlannerManager (core)** doesn't decide "should I avoid this obstacle?"
Instead:
1. Core calls module's **decision functions**
2. Module answers: "Yes, I need to run" or "No, skip me"
3. Core executes or skips based on module's answer

This is the **Responsibility Chain Pattern**.

---

## Module Decision Functions (Interface Contract)

Every module MUST implement these 3 functions:

### 1. `isExecutionRequested()` - Should this module activate?

**Location:** `scene.cpp:91-110`

```cpp
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    // Check if there's an object that should be avoided
    if (!!avoid_data_.stop_target_object) {
        return true;  // ← YES, I need to run (there's a stop target)
    }

    // Check if there's a valid shift line (avoidance path)
    if (avoid_data_.new_shift_line.empty()) {
        return false;  // ← NO, skip me (no avoidance needed)
    }

    // Check if any target object is avoidable
    return std::any_of(
        avoid_data_.target_objects.begin(),
        avoid_data_.target_objects.end(),
        [this](const auto & o) {
            return !helper_->isAbsolutelyNotAvoidable(o);  // ← Is this object avoidable?
        }
    );
}
```

**Decision Logic:**
- ✅ Return `true` if: Avoidable objects detected
- ❌ Return `false` if: No objects to avoid

### 2. `isExecutionReady()` - Is it SAFE to execute?

**Location:** `scene.cpp:112-120`

```cpp
bool StaticObstacleAvoidanceModule::isExecutionReady() const
{
    RCLCPP_DEBUG_STREAM(getLogger(), "SAFE:" << avoid_data_.safe);
    RCLCPP_DEBUG_STREAM(getLogger(), "COMFORTABLE:" << avoid_data_.comfortable);
    RCLCPP_DEBUG_STREAM(getLogger(), "VALID:" << avoid_data_.valid);
    RCLCPP_DEBUG_STREAM(getLogger(), "READY:" << avoid_data_.ready);

    // Must meet ALL conditions to execute
    return avoid_data_.safe &&        // ← No collision with other vehicles
           avoid_data_.comfortable && // ← Within jerk/acceleration limits
           avoid_data_.valid &&       // ← Path is valid
           avoid_data_.ready;         // ← All data is ready
}
```

**Decision Logic:**
- ✅ Return `true` if: Path is safe, comfortable, valid, and ready
- ❌ Return `false` if: ANY safety check fails

### 3. `getCurrentModuleState()` - What is current execution state?

**Location:** `scene.cpp:122-157`

```cpp
AvoidanceState StaticObstacleAvoidanceModule::getCurrentModuleState(
    const AvoidancePlanningData & data) const
{
    // Check if there are avoidance targets
    const bool has_avoidance_target = std::any_of(
        data.target_objects.begin(),
        data.target_objects.end(),
        [this](const auto & o) { return !helper_->isAbsolutelyNotAvoidable(o); }
    );

    if (has_avoidance_target) {
        return AvoidanceState::RUNNING;  // ← Continue avoidance
    }

    // If ego is still on shift line, keep running
    const size_t idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
    for (const auto & shift_line : path_shifter_.getShiftLines()) {
        if (shift_line.start_idx < idx && idx < shift_line.end_idx) {
            return AvoidanceState::RUNNING;  // ← Still executing shift
        }
    }

    // Check if returned to center
    const auto has_enough_distance = hasEnoughDistanceToReturnToCenter();
    const auto is_almost_centered = isAlmostCenteredLine();
    if (has_enough_distance && !is_almost_centered) {
        return AvoidanceState::RUNNING;  // ← Returning to center
    }

    return AvoidanceState::NOT_RUNNING;  // ← Done, can deactivate
}
```

**Decision Logic:**
- `RUNNING`: Objects to avoid, or still executing avoidance maneuver
- `NOT_RUNNING`: No objects, finished avoidance, returned to center

---

## How Core Uses These Functions

### In `planner_manager.cpp`:

```cpp
BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
    // 1. Update data for all modules
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(),
                  [&data](const auto & m) { m->setData(data); });

    // 2. Run modules in each slot
    for (auto & slot : planner_manager_slots_) {
        for (auto & module_manager : slot.getModuleManagers()) {

            // ========== HERE'S THE DECISION LOGIC ==========

            // Step 1: Ask module "Do you need to run?"
            if (!module->isExecutionRequested()) {
                continue;  // ← Skip this module
            }

            // Step 2: Ask module "Are you ready to run?"
            if (!module->isExecutionReady()) {
                // Module wants to run but not ready yet
                // Wait or use fallback behavior
                continue;
            }

            // Step 3: Check module state
            auto state = module->getCurrentStatus();
            if (state == ModuleStatus::IDLE) {
                // Request approval if needed (RTC)
            } else if (state == ModuleStatus::RUNNING) {
                // Execute the module
                auto output = module->plan();  // ← THIS is where scene.cpp logic runs
                result = mergeOutput(result, output);
            }
        }
    }

    return result;
}
```

---

## Complete Execution Flow (With Decision Points)

```
User drives vehicle
        ↓
Perception detects object (car stopped on road edge)
        ↓
Object data published to /perception/object_recognition/objects
        ↓
┌─────────────────────────────────────────────────────────────┐
│ behavior_path_planner_node (Core)                          │
│ - Receives object data                                     │
│ - Updates planner_data_                                    │
│ - Calls planner_manager_->run()                            │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ PlannerManager                                              │
│ - Loops through all module slots                           │
│ - For each module in Slot 2 (static_obstacle_avoidance):   │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ DECISION POINT 1: isExecutionRequested()?                   │
│                                                             │
│ StaticObstacleAvoidanceModule checks:                       │
│ 1. fillAvoidanceTargetObjects() ← Filters objects          │
│ 2. Checks if any objects are avoidable                      │
│                                                             │
│ Object detected → velocity < 1.0 m/s → stopped > 2.0s       │
│ Object on road edge → offset > 1.0m from centerline         │
│                                                             │
│ Result: return true ← "Yes, I need to run!"                 │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ DECISION POINT 2: isExecutionReady()?                       │
│                                                             │
│ StaticObstacleAvoidanceModule checks:                       │
│ 1. fillShiftLine() ← Generate avoidance path               │
│ 2. Safety check ← Collision with moving vehicles?          │
│ 3. Comfort check ← Within jerk limits?                      │
│                                                             │
│ Path generated, no collision, jerk OK                       │
│                                                             │
│ Result: return true ← "Yes, safe to execute!"               │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ DECISION POINT 3: getCurrentModuleState()?                  │
│                                                             │
│ StaticObstacleAvoidanceModule checks:                       │
│ - Still have avoidance targets? → RUNNING                   │
│ - Still on shift line? → RUNNING                            │
│ - Returned to center? → NOT_RUNNING                         │
│                                                             │
│ Result: return RUNNING ← "Keep executing!"                  │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ EXECUTION: module->plan()                                   │
│                                                             │
│ StaticObstacleAvoidanceModule::plan():                      │
│ 1. updateData() ← Update reference path                     │
│ 2. fillAvoidanceTargetObjects() ← Classify objects          │
│ 3. fillShiftLine() ← Generate lateral shift                 │
│ 4. fillEgoStatus() ← Update FSM state                       │
│ 5. Generate final path with shift lines                     │
│                                                             │
│ Output: BehaviorModuleOutput (path with avoidance)          │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ PlannerManager merges outputs                               │
│ - Combines paths from all active modules                    │
│ - Publishes final path                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## Where is the FSM Logic?

The FSM (Finite State Machine) is implemented in **multiple places**:

### 1. Module Status FSM (in Common Framework)

**File:** `scene_module_interface.hpp`

```cpp
enum class ModuleStatus {
    IDLE = 0,              // Module activated but waiting approval
    RUNNING = 1,           // Module executing
    WAITING_APPROVAL = 2,  // RTC approval needed
    SUCCESS = 3,           // Module completed successfully
    FAILURE = 4,           // Module failed
};
```

**State Transitions (in `scene_module_interface.hpp:360-410`):**

```cpp
void updateCurrentState()
{
    switch (current_state_) {
        case ModuleStatus::IDLE:
            if (isWaitingApproval()) {
                current_state_ = ModuleStatus::WAITING_APPROVAL;
            } else {
                current_state_ = ModuleStatus::RUNNING;
            }
            break;

        case ModuleStatus::RUNNING:
            if (canTransitSuccessState()) {
                current_state_ = ModuleStatus::SUCCESS;  // ← FSM transition
            }
            if (canTransitFailureState()) {
                current_state_ = ModuleStatus::FAILURE;  // ← FSM transition
            }
            break;

        case ModuleStatus::WAITING_APPROVAL:
            if (canTransitSuccessState()) {
                current_state_ = ModuleStatus::SUCCESS;
            }
            if (canTransitFailureState()) {
                current_state_ = ModuleStatus::FAILURE;
            }
            break;

        // ... other states
    }
}
```

### 2. Avoidance State FSM (in Static Obstacle Avoidance Module)

**File:** `scene.cpp:122-157`

```cpp
enum class AvoidanceState {
    NOT_RUNNING,  // No avoidance needed
    RUNNING,      // Actively avoiding
    // Note: CANCEL and SUCCEEDED are implicit through ModuleStatus
};
```

**This is where YOUR BT will replace the FSM!**

---

## Summary: Answer to Your Question

**Q: Does the core just call `StaticObstacleAvoidanceModuleManager` blindly?**

**A: NO! The execution flow is:**

1. **Core asks module**: "Do you need to run?" (`isExecutionRequested()`)
   - Module logic: Check if avoidable objects exist
   - If NO → Core skips this module

2. **Core asks module**: "Are you ready to run?" (`isExecutionReady()`)
   - Module logic: Check safety, comfort, validity
   - If NO → Core waits or uses fallback

3. **Core asks module**: "What's your state?" (`getCurrentModuleState()`)
   - Module logic: Check if still avoiding, on shift line, etc.
   - Based on state → Core decides to continue, succeed, or fail

4. **Core executes module**: `module->plan()`
   - THIS is where the actual FSM logic runs
   - Object classification, path generation, state transitions

**The module has the decision logic, NOT the core!**

The core is just an orchestrator that:
- Calls decision functions
- Respects module's answers
- Merges outputs from multiple modules

---

## For Your BT Implementation

**Key Insight:** You need to replace the decision logic in these 3 functions:

1. **`isExecutionRequested()`**
   - BT Condition Node: "HasAvoidableObjects?"

2. **`isExecutionReady()`**
   - BT Sequence: CheckSafety → CheckComfort → CheckValidity

3. **`getCurrentModuleState()`** / **`plan()`**
   - BT Tree: ObjectDetection → PathGeneration → Execution

The **core (PlannerManager) doesn't need to change** - it will still call these functions, but the implementation inside will be BT-based instead of FSM-based!

---

**End of Document**
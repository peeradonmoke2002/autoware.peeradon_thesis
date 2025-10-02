# BT Integration Strategy: Where to Implement?

**Your Question:** Do I need to create a BT planner manager (like `behavior_path_planner`) OR can I start with `behavior_path_static_obstacle_avoidance_module`?

**Answer:** **Start with the module!** You DON'T need to create a new planner manager.

---

## Understanding the Architecture

```
┌─────────────────────────────────────────────────────┐
│  behavior_path_planner (CORE/MANAGER)               │  ← Keep this as-is
│  - Loads modules as plugins                         │
│  - Calls isExecutionRequested()                     │
│  - Calls isExecutionReady()                         │
│  - Calls plan()                                     │
│  - Manages approved/candidate stacks                │
└──────────────────┬──────────────────────────────────┘
                   │
                   │ (Loads via pluginlib)
                   ↓
┌─────────────────────────────────────────────────────┐
│  static_obstacle_avoidance_module (YOUR TARGET)     │  ← Modify this!
│  - isExecutionRequested() ← Replace FSM with BT     │
│  - isExecutionReady() ← Replace FSM with BT         │
│  - plan() ← Replace FSM with BT                     │
└─────────────────────────────────────────────────────┘
```

**Key Insight:** The planner manager is just an **orchestrator**. It doesn't know (or care) if modules use FSM or BT internally. It just calls the interface functions.

---

## Option 1: Modify Existing Module (RECOMMENDED)

### What to Do:
Modify `autoware_behavior_path_static_obstacle_avoidance_module` to use BT internally

### Advantages:
- ✅ **Minimal changes** - Only change module internals
- ✅ **No manager modification** - Core stays the same
- ✅ **Easy comparison** - Can switch FSM ↔ BT with a flag
- ✅ **Faster development** - Less code to write
- ✅ **Same interface** - Works with existing system

### What Changes:
```cpp
// In static_obstacle_avoidance_module/src/scene.cpp

// OLD (FSM):
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    if (!!avoid_data_.stop_target_object) return true;
    if (avoid_data_.new_shift_line.empty()) return false;
    return std::any_of(...);  // FSM logic
}

// NEW (BT):
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    // Tick BT and return result
    return bt_executor_->shouldExecute(avoid_data_);
}
```

### Implementation Steps:

**Step 1: Add BT dependency**
```cmake
# In CMakeLists.txt
find_package(behaviortree_cpp REQUIRED)
ament_target_dependencies(... behaviortree_cpp)
```

**Step 2: Create BT executor class**
```cpp
// In include/.../bt_executor.hpp
class AvoidanceBTExecutor {
public:
    AvoidanceBTExecutor();
    bool shouldExecute(const AvoidancePlanningData& data);
    bool isReady(const AvoidancePlanningData& data);

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
};
```

**Step 3: Replace FSM logic**
```cpp
// In scene.cpp constructor:
bt_executor_ = std::make_unique<AvoidanceBTExecutor>();

// Replace FSM functions:
bool isExecutionRequested() const { return bt_executor_->shouldExecute(...); }
bool isExecutionReady() const { return bt_executor_->isReady(...); }
```

**Step 4: Keep interface same**
- Manager still calls `isExecutionRequested()`
- Manager still calls `isExecutionReady()`
- Manager still calls `plan()`
- Just the INTERNALS use BT instead of FSM

---

## Option 2: Create New BT Module (Alternative)

### What to Do:
Create a new module: `autoware_behavior_path_bt_obstacle_avoidance_module`

### Advantages:
- ✅ **Clean separation** - Old FSM stays untouched
- ✅ **Side-by-side comparison** - Can run both modules
- ✅ **Safe** - No risk of breaking existing system

### Disadvantages:
- ❌ **More work** - Need to copy entire module structure
- ❌ **Code duplication** - Copy utils, helpers, etc.
- ❌ **Plugin registration** - Need to register new plugin
- ❌ **Configuration** - New config files needed

### Implementation Steps:

**Step 1: Copy module structure**
```bash
cd ~/autoware/src/universe/autoware_universe/planning/behavior_path_planner
cp -r autoware_behavior_path_static_obstacle_avoidance_module \
      autoware_behavior_path_bt_obstacle_avoidance_module
```

**Step 2: Rename everything**
```bash
# Rename files, classes, namespace
# Change all "StaticObstacleAvoidance" → "BTObstacleAvoidance"
```

**Step 3: Register as plugin**
```xml
<!-- In plugins.xml -->
<class name="autoware::behavior_path_planner::BTObstacleAvoidanceModuleManager"
       type="autoware::behavior_path_planner::BTObstacleAvoidanceModuleManager"
       base_class_type="autoware::behavior_path_planner::SceneModuleManagerInterface">
  <description>BT-based obstacle avoidance</description>
</class>
```

**Step 4: Enable in config**
```yaml
# In default_preset.yaml
- arg:
    name: launch_bt_obstacle_avoidance
    default: "true"
```

---

## Option 3: Create New BT Planner Manager (NOT RECOMMENDED)

### What to Do:
Create `behavior_path_bt_planner` similar to `behavior_path_planner`

### Why NOT Recommended:
- ❌ **Massive work** - Reimplement entire manager
- ❌ **Duplicate infrastructure** - ROS nodes, topics, etc.
- ❌ **Integration issues** - How to switch between planners?
- ❌ **Unnecessary** - Manager doesn't need to know about BT

### Only Use If:
- You want to change the 6-step execution cycle
- You want different slot management
- You want BT to control module orchestration (not just module internals)

**For your thesis:** This is overkill! Not recommended.

---

## My Recommendation

### For Your Thesis: **Option 1 (Modify Existing Module)**

**Why:**
1. **Minimal scope** - Focus on BT vs FSM comparison
2. **Easy to test** - Can add a flag to switch FSM/BT
3. **Clear comparison** - Same module, different logic
4. **Faster development** - Less infrastructure work
5. **Clean thesis story** - "Replaced FSM with BT in this module"

### Implementation Approach:

```cpp
// In scene.hpp
class StaticObstacleAvoidanceModule : public SceneModuleInterface
{
private:
    // Add BT executor
    std::unique_ptr<AvoidanceBTExecutor> bt_executor_;

    // Keep existing FSM code (for comparison)
    bool isExecutionRequestedFSM() const;  // Old FSM logic
    bool isExecutionRequestedBT() const;   // New BT logic

    // Parameter to switch
    bool use_bt_;  // If true → BT, if false → FSM

public:
    bool isExecutionRequested() const override {
        if (use_bt_) {
            return isExecutionRequestedBT();
        } else {
            return isExecutionRequestedFSM();
        }
    }
};
```

**Configuration:**
```yaml
# In static_obstacle_avoidance.param.yaml
static_obstacle_avoidance:
  use_behavior_tree: true  # ← Switch FSM/BT
```

---

## Comparison Table

| Aspect | Option 1: Modify Module | Option 2: New Module | Option 3: New Manager |
|--------|------------------------|---------------------|----------------------|
| **Development Time** | 1-2 weeks | 2-4 weeks | 1-3 months |
| **Code Changes** | Small (one module) | Medium (new module) | Large (new system) |
| **Manager Changes** | None | Config only | Complete rewrite |
| **Risk** | Low | Very low | High |
| **Comparison** | Easy (same module) | Easy (side-by-side) | Hard (different systems) |
| **Thesis Scope** | Perfect ✅ | Acceptable | Too much ❌ |

---

## Step-by-Step: Modify Existing Module

### Phase 1: Preparation (Week 1)

**1. Backup original code**
```bash
cd ~/autoware/src/universe/autoware_universe/planning/behavior_path_planner
cp -r autoware_behavior_path_static_obstacle_avoidance_module \
      autoware_behavior_path_static_obstacle_avoidance_module.backup
```

**2. Add BehaviorTree.CPP dependency**
```bash
# Install library
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/autoware
colcon build --packages-select behaviortree_cpp
```

**3. Modify CMakeLists.txt**
```cmake
find_package(behaviortree_cpp REQUIRED)

ament_target_dependencies(
  autoware_behavior_path_static_obstacle_avoidance_module
  # ... existing dependencies ...
  behaviortree_cpp
)
```

### Phase 2: Create BT Executor (Week 2)

**1. Create header file**
```bash
touch include/autoware/behavior_path_static_obstacle_avoidance_module/bt_executor.hpp
```

**2. Define BT nodes**
```cpp
// bt_executor.hpp
#include <behaviortree_cpp/bt_factory.h>

namespace autoware::behavior_path_planner {

// Condition: Has avoidable objects?
class HasAvoidableObjects : public BT::ConditionNode {
    // Implementation
};

// Action: Classify objects
class ClassifyObjects : public BT::SyncActionNode {
    // Implementation
};

// BT Executor
class AvoidanceBTExecutor {
public:
    AvoidanceBTExecutor();
    bool shouldExecute(const AvoidancePlanningData& data);
    bool isReady(const AvoidancePlanningData& data);

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    void registerNodes();
    void createTree();
};

}
```

### Phase 3: Integration (Week 3)

**1. Modify scene.hpp**
```cpp
#include "bt_executor.hpp"

class StaticObstacleAvoidanceModule : public SceneModuleInterface {
private:
    std::unique_ptr<AvoidanceBTExecutor> bt_executor_;
    bool use_bt_;  // Parameter
};
```

**2. Modify scene.cpp**
```cpp
StaticObstacleAvoidanceModule::StaticObstacleAvoidanceModule(...) {
    // Read parameter
    use_bt_ = node.declare_parameter<bool>("use_behavior_tree", false);

    if (use_bt_) {
        bt_executor_ = std::make_unique<AvoidanceBTExecutor>();
    }
}

bool StaticObstacleAvoidanceModule::isExecutionRequested() const {
    if (use_bt_) {
        return bt_executor_->shouldExecute(avoid_data_);
    }

    // Original FSM logic (keep for comparison)
    if (!!avoid_data_.stop_target_object) return true;
    // ... rest of FSM ...
}
```

### Phase 4: Testing (Week 4)

**1. Test with FSM (baseline)**
```yaml
use_behavior_tree: false  # Use original FSM
```

**2. Test with BT**
```yaml
use_behavior_tree: true   # Use new BT
```

**3. Compare results**
- Same scenarios
- Measure: detection time, path quality, safety
- Document differences for thesis

---

## What You DON'T Need to Touch

✅ **Keep these unchanged:**
- `behavior_path_planner_node.cpp` (main node)
- `planner_manager.cpp` (manager)
- Module interface (`scene_module_interface.hpp`)
- Plugin loading system
- ROS topics/services
- Other modules (lane_change, goal_planner, etc.)

❌ **Only modify:**
- `static_obstacle_avoidance_module/src/scene.cpp` (add BT logic)
- `static_obstacle_avoidance_module/include/.../scene.hpp` (add BT executor)
- Create new: `bt_executor.hpp` and `bt_executor.cpp`

---

## File Structure After Implementation

```
autoware_behavior_path_static_obstacle_avoidance_module/
├── include/autoware/.../
│   ├── scene.hpp                    ← Modified (add bt_executor_)
│   ├── bt_executor.hpp              ← NEW (BT implementation)
│   ├── data_structs.hpp             ← Unchanged
│   ├── manager.hpp                  ← Unchanged
│   └── utils.hpp                    ← Unchanged
├── src/
│   ├── scene.cpp                    ← Modified (use BT or FSM)
│   ├── bt_executor.cpp              ← NEW (BT nodes)
│   ├── manager.cpp                  ← Unchanged
│   └── utils.cpp                    ← Unchanged
├── CMakeLists.txt                   ← Modified (add behaviortree_cpp)
├── package.xml                      ← Modified (add dependency)
└── README.md                        ← Update documentation
```

---

## Summary

### Question: Where to implement BT?

**Answer: In the existing module!**

| Approach | Recommendation |
|----------|---------------|
| ❌ Create new BT planner manager | Too much work, not needed |
| ⚠️ Create new BT module | Possible, but duplicates code |
| ✅ **Modify existing module** | **Best choice for thesis** |

### Why Modify Existing Module?

1. **Planner manager doesn't care** - It just calls interface functions
2. **Module controls logic** - FSM or BT is internal implementation
3. **Easy comparison** - Same module, switch with parameter
4. **Minimal changes** - Only module internals change
5. **Perfect for thesis** - Clear "before/after" comparison

### Next Step:

**Start modifying `static_obstacle_avoidance_module`:**
1. Add BT dependency
2. Create BT executor class
3. Add parameter to switch FSM/BT
4. Test both approaches
5. Compare for thesis

**Want help with the first step?** I can guide you through adding BehaviorTree.CPP to the module! 🚀

---

**End of Document**
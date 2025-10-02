# Behavior Path Planner Architecture: Core vs Modules

**Question:** What is the "core" that activates `autoware_behavior_path_static_obstacle_avoidance_module`?

**Answer:** The **core** is `behavior_path_planner_node` which uses `planner_manager` to load and manage ALL modules as plugins.

---

## Architecture Hierarchy

```
┌────────────────────────────────────────────────────────────────┐
│  behavior_path_planner_node (CORE NODE)                       │
│  File: behavior_path_planner_node.cpp                         │
│                                                                │
│  Responsibilities:                                             │
│  - ROS 2 node setup (subscribers/publishers)                   │
│  - Subscribe to inputs (objects, odometry, map, route, etc.)   │
│  - Publish outputs (path, turn signals, etc.)                  │
│  - Create and manage planner_manager                           │
└──────────────────┬─────────────────────────────────────────────┘
                   │
                   ↓
┌────────────────────────────────────────────────────────────────┐
│  PlannerManager (MODULE ORCHESTRATOR)                          │
│  File: planner_manager.cpp                                     │
│                                                                │
│  Responsibilities:                                             │
│  - Load modules as plugins (using pluginlib)                   │
│  - Manage module execution slots (priority)                    │
│  - Run modules in order                                        │
│  - Merge outputs from multiple modules                         │
│  - Handle module state transitions                             │
└──────────────────┬─────────────────────────────────────────────┘
                   │
                   ↓
    ┌──────────────┴───────────────┬──────────────┬─────────────┐
    │                              │              │             │
    ↓                              ↓              ↓             ↓
┌─────────────┐           ┌─────────────┐   ┌──────────┐  ┌──────────┐
│ Slot 1      │           │ Slot 2      │   │ Slot 3   │  │ Slot 4   │
│ start_      │           │ static_     │   │ goal_    │  │ dynamic_ │
│ planner     │           │ obstacle_   │   │ planner  │  │ obstacle_│
│             │           │ avoidance   │   │          │  │ avoidance│
└─────────────┘           │             │   └──────────┘  └──────────┘
                          │ lane_change │
                          │             │
                          │ avoidance_  │
                          │ by_lc       │
                          │             │
                          │ bidirectional│
                          └─────────────┘
```

---

## 1. Core Components

### 1.1 BehaviorPathPlannerNode (THE CORE)

**Location:** `autoware_behavior_path_planner/src/behavior_path_planner_node.cpp`

**What it does:**
- **Main ROS 2 node** that runs in the system
- Subscribes to inputs:
  - `/perception/object_recognition/objects`
  - `/localization/kinematic_state`
  - `/map/vector_map`
  - `/planning/mission_planning/route`
  - etc.
- Publishes outputs:
  - `~/output/path` (planned path)
  - `~/output/turn_indicators_cmd`
  - `~/debug/avoidance_debug_message_array`
  - etc.
- Creates `PlannerManager` instance
- Calls `planner_manager_->run()` in a loop

**Key code snippet (lines 87-98):**
```cpp
planner_manager_ = std::make_shared<PlannerManager>(*this);

// Load modules from parameter
for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    planner_manager_->launchScenePlugin(*this, name);  // ← Loads modules as plugins!
}

// Configure module slots (priority order)
planner_manager_->configureModuleSlot(slot_configuration);
```

---

### 1.2 PlannerManager (MODULE ORCHESTRATOR)

**Location:** `autoware_behavior_path_planner/src/planner_manager.cpp`

**What it does:**
- **Loads modules dynamically as plugins** using `pluginlib`
- Manages execution order through "slots"
- Runs each module and collects their outputs
- Merges paths from multiple modules

**Key code snippet (lines 50-71):**
```cpp
void PlannerManager::launchScenePlugin(rclcpp::Node & node, const std::string & name)
{
    if (plugin_loader_.isClassAvailable(name)) {
        const auto plugin = plugin_loader_.createSharedInstance(name);  // ← Create module instance
        plugin->init(&node);
        manager_ptrs_.push_back(plugin);  // ← Register module
        RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
    }
}
```

**Pluginlib Loader (line 37-39):**
```cpp
plugin_loader_(
    "autoware_behavior_path_planner",  // Package name
    "autoware::behavior_path_planner::SceneModuleManagerInterface"  // Base class
)
```

---

## 2. How Modules are Activated

### Step 1: Configuration File Specifies Modules

**File:** `/autoware_launch/config/planning/preset/default_preset.yaml`

```yaml
- arg:
    name: launch_static_obstacle_avoidance
    default: "true"  # ← Enable/disable this module
```

### Step 2: Launch File Passes Module List

**File:** `tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml`

```xml
<!-- If launch_static_obstacle_avoidance is true, add to module list -->
<let
  name="behavior_path_planner_launch_modules"
  value="$(eval &quot;'$(var behavior_path_planner_launch_modules)' +
         'autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager, '&quot;)"
  if="$(var launch_static_obstacle_avoidance)"
/>
```

### Step 3: Node Receives Module List as Parameter

**In `behavior_path_planner_node.cpp` (line 89-95):**
```cpp
for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    planner_manager_->launchScenePlugin(*this, name);
}
```

The parameter `launch_modules` contains:
```
[
  "autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager",
  "autoware::behavior_path_planner::LaneChangeLeftModuleManager",
  "autoware::behavior_path_planner::GoalPlannerModuleManager",
  ...
]
```

### Step 4: Pluginlib Loads Module

**How pluginlib works:**
1. Reads plugin manifest (XML file in each module)
2. Dynamically loads shared library (`.so` file)
3. Creates instance of the module's manager class
4. Calls `init()` to initialize the module

**Plugin Manifest Example:**
`autoware_behavior_path_static_obstacle_avoidance_module/plugins.xml`
```xml
<library path="autoware_behavior_path_static_obstacle_avoidance_module">
  <class name="autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager"
         type="autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager"
         base_class_type="autoware::behavior_path_planner::SceneModuleManagerInterface">
    <description>Static obstacle avoidance module manager</description>
  </class>
</library>
```

---

## 3. Module Execution Flow

### When Planning Runs:

```cpp
// In behavior_path_planner_node.cpp timer callback (~10 Hz)
void BehaviorPathPlannerNode::run()
{
    // 1. Get latest sensor data
    updatePlannerData();

    // 2. Run planner manager (which runs all modules)
    auto result = planner_manager_->run(planner_data_);  // ← THIS IS WHERE MODULES RUN

    // 3. Publish result
    path_publisher_->publish(result.path);
}
```

**Inside `planner_manager_->run()`:**
```cpp
BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
    // 1. Update data for all modules
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(),
                  [&data](const auto & m) { m->setData(data); });

    // 2. Run modules in each slot (priority order)
    for (auto & slot : planner_manager_slots_) {
        slot.run();  // ← Runs static_obstacle_avoidance, lane_change, etc.
    }

    // 3. Merge outputs
    return mergeOutputs();
}
```

---

## 4. Module Plugin Architecture

### Base Class: SceneModuleManagerInterface

All module managers inherit from this:

```cpp
class StaticObstacleAvoidanceModuleManager : public SceneModuleManagerInterface
{
public:
    void init(rclcpp::Node * node) override;

    std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
    {
        return std::make_unique<StaticObstacleAvoidanceModule>(...);
    }
};
```

### Module Lifecycle:

```
1. launchScenePlugin()
   ↓
2. plugin->init()  ← Load parameters, setup
   ↓
3. createNewSceneModuleInstance()  ← Create actual module
   ↓
4. module->updateData()  ← Update with latest data
   ↓
5. module->isExecutionRequested()  ← Check if should run
   ↓
6. module->plan()  ← Generate path
   ↓
7. module->postProcess()  ← Cleanup
```

---

## 5. Summary: What Activates Static Obstacle Avoidance?

**Full Activation Chain:**

```
1. default_preset.yaml
   launch_static_obstacle_avoidance: true
        ↓
2. behavior_planning.launch.xml
   Adds "StaticObstacleAvoidanceModuleManager" to launch_modules parameter
        ↓
3. behavior_path_planner_node.cpp (main node)
   Receives launch_modules parameter
        ↓
4. planner_manager.cpp
   launchScenePlugin("StaticObstacleAvoidanceModuleManager")
        ↓
5. pluginlib
   Loads static_obstacle_avoidance_module.so dynamically
        ↓
6. StaticObstacleAvoidanceModuleManager::init()
   Module is now registered and ready
        ↓
7. planner_manager_->run() (called every cycle)
   Module executes if conditions are met
```

---

## 6. Key Files to Understand

### Core Framework:
1. **`behavior_path_planner_node.cpp`** - Main entry point
2. **`planner_manager.cpp`** - Module orchestrator
3. **`scene_module_interface.hpp`** - Base class for all modules

### Configuration:
4. **`default_preset.yaml`** - Enable/disable modules
5. **`scene_module_manager.param.yaml`** - Module priority slots
6. **`behavior_planning.launch.xml`** - Module loading logic

### Static Obstacle Avoidance Module:
7. **`manager.cpp`** - Module manager (plugin entry point)
8. **`scene.cpp`** - Module logic (FSM, path generation)
9. **`utils.cpp`** - Helper functions (object filtering)
10. **`plugins.xml`** - Pluginlib manifest

---

## 7. For Your BT Implementation

**You have two options:**

### Option 1: Replace Module Entirely
- Create new BT-based module
- Register as plugin
- Disable old static_obstacle_avoidance module

### Option 2: Replace FSM Logic Inside Module
- Keep plugin architecture
- Replace `scene.cpp` logic with BT executor
- Maintain same interfaces

**Recommendation:** Option 2 is easier for testing
- Less infrastructure changes
- Can switch between FSM/BT easily
- Same topics/parameters

---

**End of Document**

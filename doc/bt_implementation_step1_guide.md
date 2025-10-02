# BT Implementation - Step 1: Getting Started Guide

**Goal:** Replace FSM logic in Static Obstacle Avoidance with Behavior Tree (BT)

**Date:** 2025-10-02

---

## Step 1: What You Need (Topics, Data, and Tools)

### 1.1 Required ROS 2 Topics (Input)

Your BT needs the SAME data that the current FSM uses:

| Topic Name | Message Type | Purpose | Already Available? |
|------------|-------------|---------|-------------------|
| `/perception/object_recognition/objects` | `autoware_perception_msgs/msg/PredictedObjects` | Detected objects (cars, trucks, etc.) | ✅ Yes (from perception) |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | Ego vehicle position & velocity | ✅ Yes (from localization) |
| `/map/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | HD Map data | ✅ Yes (from map loader) |
| `/planning/mission_planning/route` | `autoware_planning_msgs/msg/LaneletRoute` | Current route | ✅ Yes (from mission planner) |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Vehicle velocity | ✅ Yes (from vehicle) |

**Good News:** All topics are already available in Autoware! The module manager (`planner_data_`) provides them to your module.

---

### 1.2 Required Data Structures (Already in Autoware)

Your BT will use existing Autoware data structures:

```cpp
// Already available in planner_data_
planner_data_->dynamic_object           // Perception objects
planner_data_->self_odometry            // Ego pose
planner_data_->route_handler            // Route and map info
planner_data_->parameters               // Vehicle parameters

// Module-specific data (already exists)
avoid_data_.target_objects              // Filtered avoidable objects
avoid_data_.reference_path              // Current path
avoid_data_.safe                        // Safety flag
avoid_data_.comfortable                 // Comfort flag
```

**No new topics needed!** Everything is already there.

---

### 1.3 Required Libraries and Tools

#### A) BehaviorTree.CPP Library

**Repository:** https://github.com/BehaviorTree/BehaviorTree.CPP

**Installation:**
```bash
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
git checkout 4.6.2  # Latest stable version

# Build
cd ~/autoware
colcon build --packages-select behaviortree_cpp
source install/setup.bash
```

**Features:**
- ✅ C++ native (matches Autoware)
- ✅ ROS 2 compatible
- ✅ XML-based tree definition
- ✅ Visual editor (Groot2)
- ✅ Built-in nodes (Sequence, Fallback, etc.)

#### B) Groot2 (Visual BT Editor - Optional but Recommended)

**Download:** https://www.behaviortree.dev/groot/

**Purpose:**
- Visualize BT structure
- Edit BT in GUI
- Monitor BT execution in real-time
- Debug BT logic

```bash
# Install Groot2 (AppImage)
wget https://github.com/BehaviorTree/Groot2/releases/download/v2.0.0/Groot2-v2.0.0-x86_64.AppImage
chmod +x Groot2-v2.0.0-x86_64.AppImage
./Groot2-v2.0.0-x86_64.AppImage
```

---

## Step 1 Implementation Options

I have **TWO suggestions** for you. Choose based on your preference:

---

## Option 1: Standalone BT Testing (Recommended for Beginners)

**Approach:** Test BT logic independently BEFORE integrating with Autoware

### Pros:
- ✅ Faster development cycle
- ✅ Easy debugging
- ✅ No Autoware rebuild needed
- ✅ Can use Python or C++

### Cons:
- ❌ Needs later integration work
- ❌ Not testing with real Autoware data flow

### Implementation:

**Step 1.1: Create Standalone BT Package**

```bash
cd ~/autoware/src
ros2 pkg create bt_obstacle_avoidance_test \
  --build-type ament_cmake \
  --dependencies rclcpp behaviortree_cpp std_msgs geometry_msgs

cd bt_obstacle_avoidance_test
```

**Step 1.2: Define BT Nodes (C++)**

Create `src/bt_nodes.cpp`:

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>

using namespace BT;

// ============= CONDITION NODES =============

// Check if there are avoidable objects
class HasAvoidableObjects : public BT::ConditionNode
{
public:
    HasAvoidableObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("num_objects") };
    }

    BT::NodeStatus tick() override
    {
        auto num_objects = getInput<int>("num_objects");
        if (num_objects && num_objects.value() > 0) {
            std::cout << "✓ Avoidable objects found: " << num_objects.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout << "✗ No avoidable objects" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

// Check if avoidance path is safe
class IsPathSafe : public BT::ConditionNode
{
public:
    IsPathSafe(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<bool>("is_safe") };
    }

    BT::NodeStatus tick() override
    {
        auto is_safe = getInput<bool>("is_safe");
        if (is_safe && is_safe.value()) {
            std::cout << "✓ Path is safe" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout << "✗ Path is unsafe" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

// ============= ACTION NODES =============

// Classify objects (find avoidable ones)
class ClassifyObjects : public BT::SyncActionNode
{
public:
    ClassifyObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<int>("num_objects") };
    }

    BT::NodeStatus tick() override
    {
        // TODO: Replace with actual object classification logic
        int num_avoidable = 1; // Simulated: 1 broken-down car
        setOutput("num_objects", num_avoidable);
        std::cout << "→ Classified objects: " << num_avoidable << " avoidable" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// Generate avoidance path
class GenerateAvoidancePath : public BT::SyncActionNode
{
public:
    GenerateAvoidancePath(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<bool>("is_safe") };
    }

    BT::NodeStatus tick() override
    {
        // TODO: Replace with actual path generation logic
        bool path_safe = true; // Simulated: path is safe
        setOutput("is_safe", path_safe);
        std::cout << "→ Generated avoidance path (safe: " << path_safe << ")" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// Execute avoidance maneuver
class ExecuteAvoidance : public BT::SyncActionNode
{
public:
    ExecuteAvoidance(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override
    {
        std::cout << "→ Executing avoidance maneuver..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// ============= MAIN =============

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create BT factory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<HasAvoidableObjects>("HasAvoidableObjects");
    factory.registerNodeType<IsPathSafe>("IsPathSafe");
    factory.registerNodeType<ClassifyObjects>("ClassifyObjects");
    factory.registerNodeType<GenerateAvoidancePath>("GenerateAvoidancePath");
    factory.registerNodeType<ExecuteAvoidance>("ExecuteAvoidance");

    // Load BT from XML
    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="StaticObstacleAvoidance">
                <Sequence>
                    <ClassifyObjects num_objects="{num_objects}"/>
                    <HasAvoidableObjects num_objects="{num_objects}"/>
                    <GenerateAvoidancePath is_safe="{is_safe}"/>
                    <IsPathSafe is_safe="{is_safe}"/>
                    <ExecuteAvoidance/>
                </Sequence>
            </BehaviorTree>
        </root>
    )");

    // Enable Groot2 monitoring (optional)
    BT::Groot2Publisher publisher(tree);

    // Run BT
    std::cout << "\n=== Starting BT Execution ===\n" << std::endl;
    auto status = tree.tickWhileRunning();
    std::cout << "\n=== BT Finished with status: " << toStr(status) << " ===\n" << std::endl;

    rclcpp::shutdown();
    return 0;
}
```

**Step 1.3: Build and Test**

```bash
cd ~/autoware
colcon build --packages-select bt_obstacle_avoidance_test
source install/setup.bash

# Run standalone BT
ros2 run bt_obstacle_avoidance_test bt_test_node

# Expected output:
# === Starting BT Execution ===
# → Classified objects: 1 avoidable
# ✓ Avoidable objects found: 1
# → Generated avoidance path (safe: true)
# ✓ Path is safe
# → Executing avoidance maneuver...
# === BT Finished with status: SUCCESS ===
```

**Step 1.4: Visualize in Groot2**

```bash
# In terminal 1: Run BT with Groot2 publisher
ros2 run bt_obstacle_avoidance_test bt_test_node

# In terminal 2: Open Groot2
./Groot2-v2.0.0-x86_64.AppImage

# In Groot2:
# - Click "Connect to Server"
# - Enter: localhost:1667
# - Watch BT execute in real-time!
```

---

## Option 2: Direct Integration (Advanced)

**Approach:** Replace FSM directly in Autoware module

### Pros:
- ✅ Real integration from start
- ✅ Uses actual Autoware data
- ✅ No later integration needed

### Cons:
- ❌ Slower iteration (need to rebuild Autoware)
- ❌ Harder to debug
- ❌ Must understand Autoware build system

### Implementation:

**Step 2.1: Modify Static Obstacle Avoidance Module**

```bash
cd ~/autoware/src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module
```

**Step 2.2: Add BehaviorTree.CPP Dependency**

Edit `package.xml`:
```xml
<depend>behaviortree_cpp</depend>
```

Edit `CMakeLists.txt`:
```cmake
find_package(behaviortree_cpp REQUIRED)

ament_target_dependencies(
  autoware_behavior_path_static_obstacle_avoidance_module
  behaviortree_cpp
  # ... other dependencies
)
```

**Step 2.3: Create BT Executor Class**

Create `include/autoware/behavior_path_static_obstacle_avoidance_module/bt_executor.hpp`:

```cpp
#ifndef BT_EXECUTOR_HPP_
#define BT_EXECUTOR_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

namespace autoware::behavior_path_planner
{

class AvoidanceBTExecutor
{
public:
    AvoidanceBTExecutor();

    // Main BT execution
    bool shouldExecute(const AvoidancePlanningData& data);
    bool isReady(const AvoidancePlanningData& data);

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;

    void registerNodes();
};

} // namespace
#endif
```

**Step 2.4: Replace FSM in `scene.cpp`**

```cpp
// In StaticObstacleAvoidanceModule constructor:
bt_executor_ = std::make_unique<AvoidanceBTExecutor>();

// Replace isExecutionRequested():
bool StaticObstacleAvoidanceModule::isExecutionRequested() const
{
    // Old FSM code commented out
    // if (!!avoid_data_.stop_target_object) return true;
    // ...

    // New BT-based decision
    return bt_executor_->shouldExecute(avoid_data_);
}
```

---

## My Recommendation

**Start with Option 1 (Standalone Testing)** because:

1. **Faster Learning Curve**
   - Test BT logic without Autoware complexity
   - Iterate quickly (no 5-minute rebuilds)
   - Easy debugging with print statements

2. **Validate BT Design First**
   - Ensure BT structure is correct
   - Test all condition/action nodes
   - Verify decision logic

3. **Then Move to Option 2**
   - Once BT works standalone
   - Port nodes to Autoware module
   - Replace FSM incrementally

---

## Step 1 Deliverables (Standalone Approach)

**What to implement first:**

### 1. Basic BT Nodes (Week 1)
- ✅ `ClassifyObjects` action
- ✅ `HasAvoidableObjects` condition
- ✅ `GenerateAvoidancePath` action
- ✅ `IsPathSafe` condition
- ✅ `ExecuteAvoidance` action

### 2. BT Structure (Week 1)
```xml
<Sequence>
    <ClassifyObjects/>           <!-- Find avoidable objects -->
    <HasAvoidableObjects/>       <!-- Check if any exist -->
    <GenerateAvoidancePath/>     <!-- Plan lateral shift -->
    <IsPathSafe/>                <!-- Validate safety -->
    <ExecuteAvoidance/>          <!-- Execute maneuver -->
</Sequence>
```

### 3. Testing Scenarios (Week 2)
- Scenario 1: No objects → BT returns FAILURE
- Scenario 2: Moving object → BT returns FAILURE
- Scenario 3: Stopped object, safe path → BT returns SUCCESS
- Scenario 4: Stopped object, unsafe path → BT returns FAILURE

### 4. Integration Preparation (Week 3)
- Map BT nodes to Autoware functions
- Replace `ClassifyObjects` with actual `fillAvoidanceTargetObjects()`
- Replace `GenerateAvoidancePath` with actual `fillShiftLine()`
- Replace `IsPathSafe` with actual safety checks

---

## Quick Start Commands

```bash
# 1. Install BehaviorTree.CPP
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/autoware
colcon build --packages-select behaviortree_cpp
source install/setup.bash

# 2. Create test package
ros2 pkg create bt_obstacle_avoidance_test \
  --build-type ament_cmake \
  --dependencies rclcpp behaviortree_cpp

# 3. Copy the example code (from above) to src/bt_nodes.cpp

# 4. Build and run
colcon build --packages-select bt_obstacle_avoidance_test
source install/setup.bash
ros2 run bt_obstacle_avoidance_test bt_test_node
```

---

## Next Steps After Step 1

Once standalone BT works:

**Step 2:** Integrate with Real Autoware Data
- Subscribe to `/perception/objects`
- Use actual object classification logic
- Generate real paths

**Step 3:** Replace FSM in Module
- Modify `scene.cpp`
- Replace `isExecutionRequested()` with BT
- Replace `isExecutionReady()` with BT

**Step 4:** Test in CARLA Simulation
- Setup Autoware-CARLA bridge
- Run test scenarios
- Compare FSM vs BT performance

---

## Summary: What You Need

### Topics (Already Available):
- ✅ `/perception/object_recognition/objects`
- ✅ `/localization/kinematic_state`
- ✅ `/map/vector_map`
- ✅ `/planning/mission_planning/route`

### Libraries to Install:
- 📦 BehaviorTree.CPP (core library)
- 📦 Groot2 (optional, for visualization)

### My Suggestion:
**Option 1: Standalone Testing First**
- Faster development
- Easier debugging
- Validates BT design before full integration

### First Implementation:
5 basic BT nodes → Simple sequence tree → Test scenarios → Then integrate

---

**Want to start? I can help you:**
1. Install BehaviorTree.CPP library
2. Create the standalone test package
3. Implement the first BT nodes
4. Set up Groot2 visualization

**Which would you like to do first?** 🚀
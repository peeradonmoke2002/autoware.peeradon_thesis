# Getting Started with BT for Your Thesis

## What You Have Now ✓

### 1. **Documentation (Your Analysis)** 📚
You've created excellent documentation in `/home/peeradon/autoware/doc/`:

- **`bt_flow_object_detection_classification.md`**
  - Python BT implementation (py_trees)
  - Shows the logic flow you want
  - This is your **reference design**

- **`planning_analysis_obstacle_avoidance.md`**
  - Deep analysis of Autoware's current FSM system
  - Identifies where broken-down vehicles are handled
  - Perfect foundation for your thesis

- **`bt_integration_strategy.md`**
  - **KEY INSIGHT:** You should modify the existing module, NOT create a new planner manager
  - Clear strategy for implementation

### 2. **C++ BT Example (This Directory)** 💻
You now have a **standalone C++ test** in `/home/peeradon/autoware/test/`:

- **`simple_bt_obstacle_avoidance.cpp`**
  - Simple BT example in C++
  - Mimics your Python design
  - Includes Python-to-C++ explanations

- **`CMakeLists.txt`**
  - Build configuration

- **`README.md`**
  - Comprehensive guide
  - C++ vs Python concepts
  - Build and run instructions

## Is Your Current Code the Core? YES! ✓

**Your Python code** (`bt_flow_object_detection_classification.md`) **IS the core logic** you want!

**What it shows:**
```
Detect Objects → Filter Objects → Classify Stopped Objects
```

**This is exactly the flow you need for broken-down vehicle detection!**

The C++ example I created follows the same flow but in C++ syntax.

## Understanding the Relationship

```
┌──────────────────────────────────────────────────────────────┐
│ YOUR PYTHON BT DESIGN (doc/bt_flow_*.md)                     │
│ - Conceptual design                                          │
│ - Shows the logic                                            │
│ - Works with ROS 2 (py_trees_ros)                            │
└────────────────────┬─────────────────────────────────────────┘
                     │
                     │ Convert to C++
                     ↓
┌──────────────────────────────────────────────────────────────┐
│ C++ BT EXAMPLE (test/simple_bt_obstacle_avoidance.cpp)       │
│ - Same logic, C++ syntax                                     │
│ - Uses BehaviorTree.CPP (C++ library)                        │
│ - Standalone test (no ROS yet)                               │
└────────────────────┬─────────────────────────────────────────┘
                     │
                     │ Add ROS 2 integration
                     ↓
┌──────────────────────────────────────────────────────────────┐
│ ROS 2 BT NODE (Next step)                                    │
│ - C++ BT + ROS 2 subscribers/publishers                      │
│ - Subscribe to perception topics                             │
│ - Works with Autoware                                        │
└────────────────────┬─────────────────────────────────────────┘
                     │
                     │ Integrate with Autoware module
                     ↓
┌──────────────────────────────────────────────────────────────┐
│ AUTOWARE MODULE WITH BT                                       │
│ (autoware_behavior_path_static_obstacle_avoidance_module)    │
│ - Replace FSM in scene.cpp with BT executor                  │
│ - Keep Autoware's path planning utilities                    │
│ - Compare FSM vs BT performance                              │
└──────────────────────────────────────────────────────────────┘
```

## Your Thesis Flow (Step-by-Step)

### Phase 1: Learn C++ BT Basics (NOW) 📖

**What to do:**
1. Read the C++ example: `simple_bt_obstacle_avoidance.cpp`
2. Compare with your Python code: `doc/bt_flow_object_detection_classification.md`
3. Understand C++ concepts from README.md
4. Build and run the example

**Goal:** Understand how to write BT in C++

**Time:** 1-2 days

---

### Phase 2: Test if BT Works (NOW) 🔧

**What to do:**
1. Install BehaviorTree.CPP library
2. Build the example:
   ```bash
   cd /home/peeradon/autoware/test
   mkdir build && cd build
   cmake ..
   make
   ./simple_bt_obstacle_avoidance
   ```
3. Verify output shows 2 broken-down vehicles detected

**Goal:** Confirm BT library works on your system

**Time:** 1 hour

---

### Phase 3: Add ROS 2 Integration (NEXT) 🤖

**What to do:**
1. Create a ROS 2 node that uses BT
2. Subscribe to Autoware topics:
   - `/perception/object_recognition/objects`
   - `/localization/kinematic_state`
3. Run BT on real perception data
4. Publish detection results

**Goal:** BT working with Autoware's ROS 2 topics

**Time:** 3-5 days

**Where:** Can still work in `/home/peeradon/autoware/test` (separate from Autoware)

---

### Phase 4: Integrate with Autoware Module (LATER) 🚗

**What to do:**
1. Modify `autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`
2. Replace FSM logic with BT executor
3. Keep Autoware's utility functions (path planning, safety checks)
4. Add parameter to switch FSM/BT

**Goal:** BT running inside Autoware

**Time:** 1-2 weeks

**Where:** Inside Autoware module

---

### Phase 5: Test and Compare (FINAL) 📊

**What to do:**
1. Test in Autoware planning simulator
2. Test in CARLA simulator
3. Compare FSM vs BT:
   - Detection accuracy
   - Response time
   - Flexibility
   - Maintainability

**Goal:** Thesis results and evaluation

**Time:** 2-4 weeks

## What to Do Right Now

### Step 1: Try to Build the Example

```bash
cd /home/peeradon/autoware/test
mkdir build
cd build
cmake ..
```

**Expected outcomes:**

**✅ If CMake succeeds:**
```
-- BehaviorTree.CPP found: TRUE
-- Building simple_bt_obstacle_avoidance example
```

Then run:
```bash
make
./simple_bt_obstacle_avoidance
```

**❌ If CMake fails with "behaviortree_cpp not found":**

You need to install BehaviorTree.CPP first.

**Option A: Try apt install (Ubuntu 22.04+)**
```bash
sudo apt update
sudo apt install libbehaviortree-cpp-dev
```

**Option B: Build from source**
```bash
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake ..
make
sudo make install
```

Then go back to test directory and try cmake again.

---

### Step 2: Understand the Code

Open `simple_bt_obstacle_avoidance.cpp` and:

1. **Find the BT nodes:**
   - `DetectObjects` (line ~75)
   - `FilterObjects` (line ~120)
   - `ClassifyStoppedObjects` (line ~175)

2. **Compare with your Python code:**
   - Python `update()` method → C++ `tick()` method
   - Python `self.blackboard.get()` → C++ `getInput<Type>()`
   - Python `self.blackboard.set()` → C++ `setOutput()`

3. **See the BT structure in XML:**
   - Line ~270: XML definition
   - Shows Sequence with 3 children (like your Python tree)

---

### Step 3: Modify and Experiment

Try changing the code to understand it better:

**Experiment 1: Change thresholds**
```cpp
// In ClassifyStoppedObjects, line ~180
const double MOVING_SPEED_THRESHOLD = 1.0;  // Change to 2.0
const double MOVING_TIME_THRESHOLD = 2.0;   // Change to 5.0
```
Rebuild and see how detection changes.

**Experiment 2: Add more test data**
```cpp
// In main(), line ~290
ObjectList perception_objects = {
    // Add your own test objects here
    Object("obj_6", 0.3, 75.0, 8.0),  // New broken-down car
};
```

**Experiment 3: Add a new condition node**

Try creating a simple condition node:
```cpp
class CheckObjectInRange : public BT::ConditionNode {
public:
    CheckObjectInRange(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("distance") };
    }

    BT::NodeStatus tick() override {
        auto dist_result = getInput<double>("distance");
        if (!dist_result) return BT::NodeStatus::FAILURE;

        double dist = dist_result.value();
        if (dist >= 5.0 && dist <= 150.0) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
```

## Key Concepts to Understand

### 1. **BT is NOT the same as FSM** 🔄

**FSM (Current Autoware):**
```
State: IDLE → RUNNING → SUCCESS/FAILURE
- Fixed states
- Transition logic is hardcoded
- Difficult to add new behaviors
```

**BT (Your Goal):**
```
Tree structure: Sequence, Fallback, Parallel
- Modular nodes
- Composable behaviors
- Easy to add/remove/modify
```

### 2. **Blackboard = Shared Memory** 💾

Think of it like a global dictionary:

**Python:**
```python
blackboard = {'perception_objects': [...], 'filtered_objects': [...]}
```

**C++:**
```cpp
// Blackboard stores key-value pairs
// Nodes read/write through ports
tree.rootBlackboard()->set("perception_objects", objects);
```

### 3. **BT Execution = Tick** ⏱️

**One tick = One execution cycle:**

```
tree.tickOnce()  →  Root Sequence
                    ├── DetectObjects → SUCCESS
                    ├── FilterObjects → SUCCESS
                    └── ClassifyStoppedObjects → SUCCESS

Result: Sequence returns SUCCESS (all children succeeded)
```

In Autoware, you'll tick the tree at 10 Hz (every 100ms).

## Common Questions (Q&A)

### Q1: Do I need to understand all C++ to do this?

**A:** No! You only need to understand:
- Basic syntax (semicolons, braces, types)
- Classes and inheritance (similar to Python)
- How to use BehaviorTree.CPP library
- ROS 2 C++ basics (subscribers/publishers)

The example code has Python-to-C++ comments to help you.

---

### Q2: Should I work in test/ or directly in Autoware module?

**A:** Work in `test/` directory first!

**Advantages:**
- ✅ Fast compilation (small project)
- ✅ Easy to experiment
- ✅ No risk of breaking Autoware
- ✅ Learn C++ BT concepts safely

Once you're comfortable, then integrate with Autoware module.

---

### Q3: Is this example enough for my thesis?

**A:** This is the **foundation**, but you need to:

1. ✅ **Done:** Understand BT concepts (this example)
2. ⏳ **Next:** Add ROS 2 integration
3. ⏳ **Next:** Add path planning logic
4. ⏳ **Next:** Integrate with Autoware module
5. ⏳ **Next:** Test in simulation
6. ⏳ **Next:** Compare FSM vs BT

This example gets you started with step 1.

---

### Q4: My Python code uses py_trees, this uses BehaviorTree.CPP. Are they compatible?

**A:** They use the **same BT concepts** but different libraries:

| Feature | py_trees (Python) | BehaviorTree.CPP (C++) |
|---------|-------------------|------------------------|
| Sequence | `py_trees.composites.Sequence` | `<Sequence>` in XML |
| Condition | `py_trees.behaviour.Behaviour` | `BT::ConditionNode` |
| Action | `py_trees.behaviour.Behaviour` | `BT::SyncActionNode` |
| Blackboard | `blackboard.set/get()` | Input/Output ports |

The **logic** translates directly, just different syntax.

---

### Q5: Should I use XML or C++ code to define the tree?

**A:** For learning, both are fine. For Autoware integration:

**XML (Recommended for thesis):**
```xml
<Sequence>
    <DetectObjects />
    <FilterObjects />
</Sequence>
```
✅ Easy to visualize
✅ Can edit without recompiling
✅ Can use Groot2 visual editor

**C++ Code (Alternative):**
```cpp
auto root = std::make_shared<Sequence>("Root");
root->addChild(std::make_shared<DetectObjects>());
root->addChild(std::make_shared<FilterObjects>());
```
✅ More flexible
✅ Dynamic tree construction

For your thesis, XML is better for demonstrating BT structure visually.

## Next Steps Summary

### Immediate (Today):
1. ✅ Read this document
2. ✅ Read README.md
3. ✅ Try to build the example
4. ✅ Understand the C++ code structure

### This Week:
1. ⏳ Get BT example working
2. ⏳ Modify test data and thresholds
3. ⏳ Add your own simple BT node
4. ⏳ Compare with your Python code

### Next Week:
1. ⏳ Create ROS 2 node with BT
2. ⏳ Subscribe to Autoware perception topics
3. ⏳ Test with Autoware simulator

### This Month:
1. ⏳ Integrate BT into Autoware module
2. ⏳ Add path planning logic
3. ⏳ Test broken-down vehicle scenarios

## Need Help?

If you get stuck:

1. **Build errors:** Check if BehaviorTree.CPP is installed
2. **C++ syntax errors:** Read the Python-to-C++ comments in the code
3. **Concept questions:** Compare with your Python code in `doc/`
4. **Integration questions:** Re-read `bt_integration_strategy.md`

## Summary

**You asked: "Is the current code the core of what I want?"**

**Answer: YES! ✓**

Your documentation shows:
- ✅ You understand Autoware's architecture
- ✅ You know where to integrate BT (modify existing module)
- ✅ You have the logic flow designed (Python BT)

What you need now:
- ✅ Learn C++ BT syntax (this example helps!)
- ⏳ Convert your Python design to C++
- ⏳ Integrate with Autoware

**Start with the simple example, then gradually add complexity.**

Good luck with your thesis! 🎓🚗

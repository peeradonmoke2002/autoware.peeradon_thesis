# Simple BT Obstacle Avoidance Test

## Overview

This is a **standalone C++ Behavior Tree (BT) example** to test obstacle detection and classification for broken-down vehicles. This code demonstrates the core BT concepts you'll need for your thesis.

**Purpose:**
- Learn BT concepts in C++ (coming from Python background)
- Test if BehaviorTree.CPP works correctly
- Understand the flow: Detect → Filter → Classify
- Prototype before integrating with Autoware

## What This Example Does

```
Input: List of 5 objects (cars) with velocity, distance, stopped_duration

       ↓

[BT Root Sequence]
├── DetectObjects (Condition)
│   └── Check if any objects exist
├── FilterObjects (Action)
│   └── Filter by distance range (5m - 150m)
└── ClassifyStoppedObjects (Action)
    └── Classify stopped objects (velocity < 1.0 m/s, stopped > 2.0s)

       ↓

Output: 2 broken-down vehicles detected (obj_2 at 30m, obj_3 at 120m)
```

## C++ vs Python Concepts (For Python Developers)

### 1. **Classes and Inheritance**

**Python:**
```python
class DetectObjects(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        return py_trees.common.Status.SUCCESS
```

**C++:**
```cpp
class DetectObjects : public BT::ConditionNode {
public:
    DetectObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override {
        return BT::NodeStatus::SUCCESS;
    }
};
```

**Key Differences:**
- C++ uses `:` for inheritance (Python uses parentheses)
- C++ uses `::` to access class members (Python uses `.`)
- C++ requires explicit types (`std::string`, `const`, etc.)
- C++ uses `override` keyword to indicate method overriding
- C++ method implementation inside class uses `{}`

### 2. **Blackboard (Shared Data Storage)**

**Python:**
```python
# Write to blackboard
self.blackboard.set('perception_objects', objects)

# Read from blackboard
objects = self.blackboard.get('perception_objects')
```

**C++:**
```cpp
// Write to blackboard (output port)
setOutput("perception_objects", objects);

// Read from blackboard (input port)
auto objects_result = getInput<ObjectList>("perception_objects");
if (objects_result) {
    ObjectList objects = objects_result.value();
}
```

**Key Differences:**
- C++ uses **ports** (InputPort/OutputPort) to define blackboard access
- C++ returns `std::optional` (need to check if value exists)
- C++ requires explicit type in angle brackets: `<ObjectList>`

### 3. **Lists and Iteration**

**Python:**
```python
objects = [Object("id1", 10.0), Object("id2", 5.0)]

for obj in objects:
    print(obj.velocity)

filtered = [obj for obj in objects if obj.velocity > 5.0]
```

**C++:**
```cpp
std::vector<Object> objects = {
    Object("id1", 10.0),
    Object("id2", 5.0)
};

for (const auto& obj : objects) {
    std::cout << obj.velocity << "\n";
}

std::vector<Object> filtered;
for (const auto& obj : objects) {
    if (obj.velocity > 5.0) {
        filtered.push_back(obj);
    }
}
```

**Key Differences:**
- Python `list` → C++ `std::vector`
- Python `for obj in list:` → C++ `for (const auto& obj : list)`
- Python `list.append()` → C++ `vector.push_back()`
- C++ requires `const auto&` for efficient iteration (avoids copying)
- C++ needs `{}` for initialization lists

### 4. **Data Structures**

**Python:**
```python
@dataclass
class Object:
    id: str
    velocity: float
    distance: float
```

**C++:**
```cpp
struct Object {
    std::string id;
    double velocity;
    double distance;

    // Constructor (like __init__)
    Object(std::string id, double vel, double dist)
        : id(id), velocity(vel), distance(dist) {}
};
```

**Key Differences:**
- Python `str` → C++ `std::string`
- Python `float` → C++ `double`
- C++ requires constructor definition
- C++ uses `:` initializer list (efficient initialization)

### 5. **Types and Variables**

**Python:**
```python
name = "test"              # Type inferred
count = 5                  # Type inferred
speed = 10.5               # Type inferred

def process(data: List[Object]) -> bool:
    return True
```

**C++:**
```cpp
std::string name = "test";     // Explicit type required
int count = 5;                 // Explicit type required
double speed = 10.5;           // Explicit type required

bool process(std::vector<Object> data) {
    return true;
}
```

**Key Differences:**
- C++ requires explicit types for all variables
- C++ can use `auto` for type inference (but still statically typed)
- Python `List[Object]` → C++ `std::vector<Object>`

### 6. **None/Null Checking**

**Python:**
```python
objects = blackboard.get('objects')
if objects is None:
    return Status.FAILURE
```

**C++:**
```cpp
auto objects_result = getInput<ObjectList>("objects");
if (!objects_result) {  // Check if value exists
    return BT::NodeStatus::FAILURE;
}

ObjectList objects = objects_result.value();
```

**Key Differences:**
- Python `None` → C++ `nullptr` or `std::optional`
- C++ uses `!result` to check if optional is empty
- C++ uses `.value()` to extract value from optional

### 7. **Printing Output**

**Python:**
```python
print(f"Detected {len(objects)} objects")
print(f"Object {obj.id} at {obj.distance}m")
```

**C++:**
```cpp
std::cout << "Detected " << objects.size() << " objects\n";
std::cout << "Object " << obj.id << " at " << obj.distance << "m\n";
```

**Key Differences:**
- Python `print()` → C++ `std::cout <<`
- Python f-strings → C++ stream insertion (`<<`)
- Python auto newline → C++ manual `\n` or `std::endl`
- C++ uses `.size()` instead of Python `len()`

## Building and Running

### Prerequisites

You need **BehaviorTree.CPP** library installed.

**Option 1: Install from apt (Ubuntu 22.04+)**
```bash
sudo apt update
sudo apt install libbehaviortree-cpp-dev
```

**Option 2: Build from source (if not available in apt)**
```bash
cd ~/autoware/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake ..
make
sudo make install
```

### Build the Example

```bash
cd /home/peeradon/autoware/test

# Create build directory
mkdir build
cd build

# Configure and build
cmake ..
make

# Run the example
./simple_bt_obstacle_avoidance
```

### Expected Output

```
========================================
Simple BT Obstacle Avoidance Test
========================================

Test Data: 5 objects
- obj_1: Moving car (15.0 m/s) at 50m
- obj_2: Stopped car (0.5 m/s, 3.5s) at 30m ← BROKEN-DOWN
- obj_3: Stopped car (0.2 m/s, 5.0s) at 120m ← BROKEN-DOWN
- obj_4: Moving car (20.0 m/s) at 200m (too far)
- obj_5: Stopped car (0.8 m/s, 1.5s) at 3m (too close)

Ticking BT...
========================================

[DetectObjects] ✓ Detected 5 objects
[FilterObjects] ✓ Filtered to 3/5 objects
[ClassifyStoppedObjects] Object obj_2 stopped for 3.5s at 30m
[ClassifyStoppedObjects] Object obj_3 stopped for 5s at 120m
[ClassifyStoppedObjects] ✓ Classified 2 stopped objects out of 3 filtered

========================================
BT Status: SUCCESS ✓

Final Results:
- Stopped objects detected: 2

Broken-down vehicles to avoid:
  → obj_2 at 30m (velocity: 0.5 m/s, stopped: 3.5s)
  → obj_3 at 120m (velocity: 0.2 m/s, stopped: 5s)

========================================
Expected Result:
- Should detect 2 broken-down vehicles (obj_2, obj_3)
- obj_1: filtered out (moving)
- obj_4: filtered out (too far)
- obj_5: filtered out (stopped < 2s)
========================================
```

## BT Node Types Explained

### 1. **ConditionNode** (Check if something is true)

**Purpose:** Returns SUCCESS or FAILURE based on a condition
**Example:** `DetectObjects` checks if objects exist

**Python equivalent:**
```python
class DetectObjects(py_trees.behaviour.Behaviour):
    def update(self):
        if condition_is_true:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
```

### 2. **SyncActionNode** (Do work synchronously)

**Purpose:** Performs an action and completes immediately
**Example:** `FilterObjects`, `ClassifyStoppedObjects`

**Python equivalent:**
```python
class FilterObjects(py_trees.behaviour.Behaviour):
    def update(self):
        # Do work here
        self.blackboard.set('result', result)
        return py_trees.common.Status.SUCCESS
```

### 3. **Sequence** (All children must succeed)

**Purpose:** Executes children in order, fails if any child fails
**Example:** Root sequence (Detect → Filter → Classify)

**Python equivalent:**
```python
root = py_trees.composites.Sequence(name="Root")
root.add_children([node1, node2, node3])
```

### 4. **Fallback/Selector** (Try until one succeeds) - Not used in this example

**Purpose:** Executes children until one succeeds
**Example:** Try multiple avoidance strategies

**Python equivalent:**
```python
fallback = py_trees.composites.Selector(name="TryStrategies")
fallback.add_children([strategy1, strategy2, strategy3])
```

## Next Steps for Your Thesis

### 1. **Understand This Example First**
- Read the code with Python-to-C++ comments
- Build and run it
- Modify test data and see results
- Add your own condition/action nodes

### 2. **Add ROS 2 Integration**
- Replace simulated data with ROS 2 subscribers
- Subscribe to `/perception/object_recognition/objects`
- Subscribe to `/localization/kinematic_state`
- Publish avoidance decisions

### 3. **Integrate with Autoware Module**
- Modify `autoware_behavior_path_static_obstacle_avoidance_module`
- Replace FSM logic in `scene.cpp` with BT executor
- Keep Autoware's utility functions (path planning, safety checks)
- Add parameter to switch between FSM/BT

### 4. **Expand BT Complexity**
- Add more sophisticated filtering logic
- Add path planning nodes
- Add safety validation nodes
- Add fallback strategies (yield, stop)

### 5. **Test in Simulation**
- Test with Autoware planning simulator
- Test with CARLA simulator
- Compare FSM vs BT performance

## Key Takeaways

### C++ Memory Management (Important!)

**Python:**
- Automatic garbage collection
- No need to worry about memory

**C++:**
- Need to be careful with pointers and references
- Use `const auto&` when iterating to avoid copying
- BehaviorTree.CPP handles most memory management for you

### C++ Compilation

**Python:**
- Interpreted language
- Run directly: `python script.py`
- Errors at runtime

**C++:**
- Compiled language
- Build first: `cmake .. && make`
- Errors at compile time (safer!)

### Type Safety

**Python:**
- Dynamic typing
- Type hints are optional

**C++:**
- Static typing
- All types must be declared
- Catches type errors at compile time

## Debugging Tips

### 1. **Compilation Errors**

**Common errors for Python developers:**

```cpp
// ERROR: Forgot semicolon
std::cout << "Hello"  // ❌ Missing semicolon

// CORRECT:
std::cout << "Hello"; // ✓
```

```cpp
// ERROR: Wrong type
std::vector<Object> objects = 5;  // ❌ Can't assign int to vector

// CORRECT:
std::vector<Object> objects;      // ✓ Empty vector
```

```cpp
// ERROR: Forgot to dereference optional
auto result = getInput<ObjectList>("objects");
ObjectList objects = result;  // ❌ Can't assign optional to ObjectList

// CORRECT:
auto result = getInput<ObjectList>("objects");
if (result) {
    ObjectList objects = result.value();  // ✓ Extract value
}
```

### 2. **Runtime Errors**

**Check blackboard data exists:**
```cpp
auto result = getInput<ObjectList>("objects");
if (!result) {
    std::cout << "ERROR: objects not found in blackboard\n";
    return BT::NodeStatus::FAILURE;
}
```

**Print for debugging:**
```cpp
std::cout << "DEBUG: objects.size() = " << objects.size() << "\n";
```

## Questions?

If you get stuck:
1. Read compiler error messages carefully (they're verbose but helpful)
2. Check if BehaviorTree.CPP is installed: `dpkg -l | grep behaviortree`
3. Check CMake finds the library: Look for "BehaviorTree.CPP found: TRUE"
4. Compare with Python equivalent in comments

## References

- **BehaviorTree.CPP Documentation:** https://www.behaviortree.dev/
- **BehaviorTree.CPP GitHub:** https://github.com/BehaviorTree/BehaviorTree.CPP
- **Your Python BT Flow:** `/home/peeradon/autoware/doc/bt_flow_object_detection_classification.md`
- **Autoware Planning Analysis:** `/home/peeradon/autoware/doc/planning_analysis_obstacle_avoidance.md`

## License

This is a test/educational example for thesis work.

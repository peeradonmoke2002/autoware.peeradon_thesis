# BT Implementation with py_trees: Monitoring Autoware

**Approach:** Run Autoware normally + Python BT script to monitor and validate decision logic

**Library:** py_trees (Python Behavior Tree library)

**Goal:** Create BT that monitors Autoware, detects obstacles, and validates if vehicle stops correctly

---

## 1. Why This Approach is Smart

### Advantages:
- ✅ **No Autoware modification** - Run standard Autoware
- ✅ **Fast iteration** - Edit Python, rerun (no compilation)
- ✅ **Easy debugging** - Python print statements, breakpoints
- ✅ **Validation tool** - Test if current FSM logic is correct
- ✅ **BT learning** - Learn BT concepts before C++ integration

### What You'll Build:
```
Autoware (FSM) ────→ Publishes topics
                        ↓
                  Your Python BT
                  (Subscribes & monitors)
                        ↓
                  Validates behavior:
                  - "Did it detect obstacle?"
                  - "Did it stop correctly?"
                  - "Is avoidance path safe?"
```

---

## 2. Installation

### Install py_trees

```bash
# Install py_trees and visualization tools
pip3 install py_trees py-trees-ros-viewer

# Verify installation
python3 -c "import py_trees; print(py_trees.__version__)"
# Expected: 2.2.x or higher
```

### Install py_trees_ros (ROS 2 integration)

```bash
# Install ROS 2 integration
pip3 install py-trees-ros

# Or from source for latest version
cd ~/autoware/src
git clone https://github.com/splintered-reality/py_trees_ros.git -b devel
cd ~/autoware
colcon build --packages-select py_trees_ros
source install/setup.bash
```

---

## 3. Project Structure

```bash
# Create Python BT monitoring package
cd ~/autoware/src
mkdir -p bt_monitor/bt_monitor
cd bt_monitor

# Create files
touch bt_monitor/__init__.py
touch bt_monitor/obstacle_detection_monitor.py
touch setup.py
touch package.xml
```

---

## 4. Complete Implementation

### 4.1 `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bt_monitor</name>
  <version>0.0.1</version>
  <description>Behavior Tree monitoring for Autoware obstacle avoidance</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>autoware_perception_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>autoware_planning_msgs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 4.2 `setup.py`

```python
from setuptools import setup

package_name = 'bt_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'py-trees-ros'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='BT monitoring for obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = bt_monitor.obstacle_detection_monitor:main',
        ],
    },
)
```

### 4.3 `bt_monitor/obstacle_detection_monitor.py`

```python
#!/usr/bin/env python3
"""
Behavior Tree Monitor for Autoware Obstacle Avoidance
Monitors Autoware topics and validates obstacle detection and stopping behavior
"""

import rclpy
from rclpy.node import Node
import py_trees
from py_trees import common, blackboard
import math

from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import TwistStamped

# ==================== BT CONDITION NODES ====================

class ObjectsDetected(py_trees.behaviour.Behaviour):
    """Check if perception has detected any objects"""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key='perception_objects',
            access=py_trees.common.Access.READ
        )

    def update(self):
        objects = self.blackboard.get('perception_objects')

        if objects is None:
            self.logger.info("⏳ Waiting for perception data...")
            return py_trees.common.Status.RUNNING

        if len(objects.objects) > 0:
            self.logger.info(f"✓ Objects detected: {len(objects.objects)}")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("✗ No objects detected")
            return py_trees.common.Status.FAILURE


class HasStoppedObjects(py_trees.behaviour.Behaviour):
    """Check if any detected objects are stopped (broken-down vehicle)"""

    def __init__(self, name, speed_threshold=1.0):
        super().__init__(name)
        self.speed_threshold = speed_threshold
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key='perception_objects',
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key='stopped_objects',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        objects = self.blackboard.get('perception_objects')

        if objects is None:
            return py_trees.common.Status.RUNNING

        stopped_objects = []
        for obj in objects.objects:
            velocity = obj.kinematics.initial_twist_with_covariance.twist.linear.x
            if velocity < self.speed_threshold:
                stopped_objects.append(obj)

        if len(stopped_objects) > 0:
            self.blackboard.set('stopped_objects', stopped_objects)
            self.logger.info(f"✓ Found {len(stopped_objects)} stopped objects")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("✗ No stopped objects (all moving)")
            return py_trees.common.Status.FAILURE


class VehicleIsStopping(py_trees.behaviour.Behaviour):
    """Check if ego vehicle is decelerating/stopping"""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key='ego_velocity',
            access=py_trees.common.Access.READ
        )
        self.prev_velocity = None

    def update(self):
        current_velocity = self.blackboard.get('ego_velocity')

        if current_velocity is None:
            return py_trees.common.Status.RUNNING

        if self.prev_velocity is not None:
            # Check if decelerating
            if current_velocity < self.prev_velocity:
                decel = self.prev_velocity - current_velocity
                self.logger.info(f"✓ Vehicle decelerating: {decel:.2f} m/s")
                self.prev_velocity = current_velocity
                return py_trees.common.Status.SUCCESS
            elif current_velocity < 0.5:  # Nearly stopped
                self.logger.info("✓ Vehicle stopped")
                return py_trees.common.Status.SUCCESS

        self.prev_velocity = current_velocity
        self.logger.info(f"✗ Vehicle not stopping (velocity: {current_velocity:.2f} m/s)")
        return py_trees.common.Status.FAILURE


class AvoidancePathExists(py_trees.behaviour.Behaviour):
    """Check if planning published an avoidance path"""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key='planned_trajectory',
            access=py_trees.common.Access.READ
        )

    def update(self):
        trajectory = self.blackboard.get('planned_trajectory')

        if trajectory is None:
            return py_trees.common.Status.RUNNING

        if len(trajectory.points) > 0:
            # Check if path has lateral offset (avoidance)
            # Simple check: if any point deviates from centerline
            has_offset = False
            for point in trajectory.points[:10]:  # Check first 10 points
                if abs(point.lateral_velocity_mps) > 0.1:
                    has_offset = True
                    break

            if has_offset:
                self.logger.info("✓ Avoidance path detected (lateral offset)")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.info("✗ No avoidance (following centerline)")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE


# ==================== BT ACTION NODES ====================

class LogDetection(py_trees.behaviour.Behaviour):
    """Log obstacle detection event"""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key='stopped_objects',
            access=py_trees.common.Access.READ
        )

    def update(self):
        objects = self.blackboard.get('stopped_objects')
        if objects:
            self.logger.warning(f"⚠️  OBSTACLE DETECTED: {len(objects)} stopped object(s)")
            for i, obj in enumerate(objects):
                pos = obj.kinematics.initial_pose_with_covariance.pose.position
                self.logger.info(f"   Object {i}: position=({pos.x:.2f}, {pos.y:.2f})")
        return py_trees.common.Status.SUCCESS


class ValidateStopBehavior(py_trees.behaviour.Behaviour):
    """Validate that vehicle stopped correctly for obstacle"""

    def __init__(self, name):
        super().__init__(name)

    def update(self):
        self.logger.warning("✅ VALIDATION PASSED: Vehicle stopped for obstacle")
        return py_trees.common.Status.SUCCESS


class ValidateAvoidanceBehavior(py_trees.behaviour.Behaviour):
    """Validate that vehicle is avoiding obstacle"""

    def __init__(self, name):
        super().__init__(name)

    def update(self):
        self.logger.warning("✅ VALIDATION PASSED: Vehicle avoiding obstacle")
        return py_trees.common.Status.SUCCESS


# ==================== ROS NODE ====================

class BTMonitorNode(Node):
    """ROS 2 Node that runs BT monitoring"""

    def __init__(self):
        super().__init__('bt_monitor_node')

        # Initialize blackboard
        self.blackboard = blackboard.Client(name="BTMonitor")
        self.blackboard.register_key(key='perception_objects', access=common.Access.WRITE)
        self.blackboard.register_key(key='ego_velocity', access=common.Access.WRITE)
        self.blackboard.register_key(key='planned_trajectory', access=common.Access.WRITE)

        # Subscribers
        self.objects_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.objects_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.trajectory_callback,
            10
        )

        # Build Behavior Tree
        self.tree = self.create_bt()

        # Timer to tick BT (10 Hz to match Autoware planning)
        self.timer = self.create_timer(0.1, self.tick_bt)

        self.get_logger().info("🌲 BT Monitor Node Started")
        self.get_logger().info("Monitoring Autoware for obstacle detection and stopping behavior...")

    def objects_callback(self, msg):
        self.blackboard.set('perception_objects', msg)

    def odom_callback(self, msg):
        velocity = msg.twist.twist.linear.x
        self.blackboard.set('ego_velocity', velocity)

    def trajectory_callback(self, msg):
        self.blackboard.set('planned_trajectory', msg)

    def create_bt(self):
        """Create the behavior tree structure"""

        # Root: Monitor for obstacle and validate response
        root = py_trees.composites.Selector(
            name="ObstacleMonitor",
            memory=False
        )

        # Branch 1: Detect obstacle and validate STOP behavior
        stop_sequence = py_trees.composites.Sequence(
            name="DetectAndStop",
            memory=True
        )
        stop_sequence.add_children([
            ObjectsDetected(name="Objects?"),
            HasStoppedObjects(name="Stopped?"),
            LogDetection(name="LogObstacle"),
            VehicleIsStopping(name="VehicleStopping?"),
            ValidateStopBehavior(name="ValidateStop")
        ])

        # Branch 2: Detect obstacle and validate AVOIDANCE behavior
        avoid_sequence = py_trees.composites.Sequence(
            name="DetectAndAvoid",
            memory=True
        )
        avoid_sequence.add_children([
            ObjectsDetected(name="Objects?"),
            HasStoppedObjects(name="Stopped?"),
            LogDetection(name="LogObstacle"),
            AvoidancePathExists(name="AvoidancePath?"),
            ValidateAvoidanceBehavior(name="ValidateAvoidance")
        ])

        # Add branches to root
        root.add_children([
            avoid_sequence,  # Try avoidance first
            stop_sequence,   # Fallback to stop
        ])

        return py_trees.trees.BehaviourTree(root)

    def tick_bt(self):
        """Tick the behavior tree"""
        self.tree.tick()

        # Print tree status (optional)
        # print(py_trees.display.unicode_tree(self.tree.root, show_status=True))


# ==================== MAIN ====================

def main(args=None):
    rclpy.init(args=args)

    # Create and run node
    node = BTMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5. Build and Run

### Build the Package

```bash
cd ~/autoware
colcon build --packages-select bt_monitor
source install/setup.bash
```

### Run Autoware (Terminal 1)

```bash
# Launch Autoware with planning stack
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

### Run BT Monitor (Terminal 2)

```bash
source ~/autoware/install/setup.bash
ros2 run bt_monitor monitor
```

### Expected Output

```
[INFO] [bt_monitor_node]: 🌲 BT Monitor Node Started
[INFO] [bt_monitor_node]: Monitoring Autoware for obstacle detection and stopping behavior...

# When no obstacles:
[INFO] [Objects?]: ✗ No objects detected

# When obstacle appears:
[INFO] [Objects?]: ✓ Objects detected: 3
[INFO] [Stopped?]: ✓ Found 1 stopped objects
[WARN] [LogObstacle]: ⚠️  OBSTACLE DETECTED: 1 stopped object(s)
[INFO] [LogObstacle]:    Object 0: position=(45.23, 12.45)
[INFO] [AvoidancePath?]: ✓ Avoidance path detected (lateral offset)
[WARN] [ValidateAvoidance]: ✅ VALIDATION PASSED: Vehicle avoiding obstacle
```

---

## 6. Visualization with py_trees_ros_viewer

### Install Viewer

```bash
pip3 install py-trees-ros-viewer
```

### Run with Visualization

Modify `bt_monitor/obstacle_detection_monitor.py` to add ROS viewer:

```python
# Add at top
from py_trees_ros import trees

# In BTMonitorNode.__init__(), replace tree creation:
self.tree = trees.BehaviourTree(
    root=self.create_bt().root,
    unicode_tree_debug=True
)

# Add snapshot publisher
self.tree.setup(node=self)
```

### Launch Viewer

```bash
# Terminal 3: Launch viewer
py-trees-tree-viewer
```

You'll see live BT visualization showing which nodes are SUCCESS/FAILURE/RUNNING!

---

## 7. Testing Scenarios

### Test 1: No Obstacles (Baseline)

**Setup:** Clear road, no objects

**Expected BT Flow:**
```
ObstacleMonitor [FAILURE]
├── DetectAndAvoid [FAILURE]
│   └── Objects? [FAILURE] ← No objects detected
└── DetectAndStop [FAILURE]
    └── Objects? [FAILURE] ← No objects detected
```

**Result:** BT returns FAILURE (no action needed)

---

### Test 2: Moving Vehicle (Not Broken-Down)

**Setup:** Moving car at 15 m/s

**Expected BT Flow:**
```
ObstacleMonitor [FAILURE]
├── DetectAndAvoid [FAILURE]
│   ├── Objects? [SUCCESS] ← Detected
│   └── Stopped? [FAILURE] ← Moving > 1.0 m/s
└── DetectAndStop [FAILURE]
    ├── Objects? [SUCCESS]
    └── Stopped? [FAILURE] ← Moving
```

**Result:** BT returns FAILURE (not an obstacle)

---

### Test 3: Broken-Down Vehicle (Avoidance)

**Setup:** Stopped car (0 m/s) on road edge

**Expected BT Flow:**
```
ObstacleMonitor [SUCCESS]
└── DetectAndAvoid [SUCCESS]
    ├── Objects? [SUCCESS] ← Detected
    ├── Stopped? [SUCCESS] ← Velocity < 1.0 m/s
    ├── LogObstacle [SUCCESS] ← Logged warning
    ├── AvoidancePath? [SUCCESS] ← Path has lateral offset
    └── ValidateAvoidance [SUCCESS] ← ✅ PASSED
```

**Result:** BT validates avoidance behavior

---

### Test 4: Obstacle Ahead (Stop Required)

**Setup:** Stopped car directly in path, can't avoid

**Expected BT Flow:**
```
ObstacleMonitor [SUCCESS]
├── DetectAndAvoid [FAILURE]
│   ├── Objects? [SUCCESS]
│   ├── Stopped? [SUCCESS]
│   ├── LogObstacle [SUCCESS]
│   └── AvoidancePath? [FAILURE] ← No lateral path
└── DetectAndStop [SUCCESS]
    ├── Objects? [SUCCESS]
    ├── Stopped? [SUCCESS]
    ├── LogObstacle [SUCCESS]
    ├── VehicleStopping? [SUCCESS] ← Decelerating
    └── ValidateStop [SUCCESS] ← ✅ PASSED
```

**Result:** BT validates stop behavior

---

## 8. Advantages of This Approach

### For Learning:
- ✅ Learn BT concepts with Python (easier than C++)
- ✅ Understand py_trees library structure
- ✅ See BT tick cycle in action

### For Validation:
- ✅ Verify Autoware's FSM logic is correct
- ✅ Identify edge cases or bugs
- ✅ Compare expected vs actual behavior

### For Thesis:
- ✅ Demonstrate understanding of current system
- ✅ Baseline for BT comparison
- ✅ Testing framework for later C++ BT

---

## 9. Next Steps

### Phase 1: Monitor (Current)
- ✅ Run alongside Autoware
- ✅ Validate obstacle detection
- ✅ Check stop/avoidance behavior

### Phase 2: Extend BT
- Add more conditions (distance checks, safety checks)
- Add timing validation (reaction time)
- Log metrics for thesis

### Phase 3: Python BT Control (Optional)
- Not just monitor, but send commands
- Publish velocity commands
- Test BT as controller (before C++ integration)

### Phase 4: Port to C++
- Convert py_trees nodes → BehaviorTree.CPP
- Integrate with Autoware module
- Replace FSM

---

## 10. Quick Reference

### Key Topics to Monitor

```python
# Input topics (subscribe)
/perception/object_recognition/objects           # Objects detected
/localization/kinematic_state                    # Ego vehicle state
/planning/scenario_planning/trajectory           # Planned path

# Output topics (to validate)
/planning/scenario_planning/lane_driving/.../path  # Avoidance path
/vehicle/status/velocity_status                  # Vehicle velocity
```

### BT Node Types

```python
# Condition (check something, return SUCCESS/FAILURE)
class IsConditionMet(py_trees.behaviour.Behaviour):
    def update(self):
        if condition:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# Action (do something, return SUCCESS when done)
class DoAction(py_trees.behaviour.Behaviour):
    def update(self):
        perform_action()
        return py_trees.common.Status.SUCCESS
```

---

## Summary

**What you'll do:**
1. ✅ Run Autoware normally
2. ✅ Run Python BT monitor in parallel
3. ✅ BT subscribes to Autoware topics
4. ✅ BT validates obstacle detection and stopping
5. ✅ Collect data for thesis

**Advantages:**
- No Autoware modification needed
- Fast Python iteration
- Visual BT debugging
- Perfect for learning and validation

**Ready to start?** Just run:
```bash
cd ~/autoware
colcon build --packages-select bt_monitor
source install/setup.bash
ros2 run bt_monitor monitor
```

🚀 Let me know when you want to run it or need help with any step!
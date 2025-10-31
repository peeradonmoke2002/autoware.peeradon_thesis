# BT Flow: Object Detection & Classification

This document details the Behavior Tree implementation for **Object Detection & Classification** phase.

**Scope**: Detect objects from perception and classify which ones require stopping.

---

## BT Structure Overview

```
Root (Sequence)
  └─ DetectAndClassify (Sequence)
       ├─ SubscribeTopics (Parallel - All must succeed)
       │    ├─ PerceptionSubscriber
       │    └─ OdometrySubscriber
       ├─ DetectObjects (Condition)
       ├─ FilterObjects (Action)
       └─ ClassifyStoppedObjects (Action)
```

---

## Detailed Node Descriptions

### 1. SubscribeTopics (Parallel)

**Type**: `py_trees.composites.Parallel`
**Policy**: `SuccessOnAll()` - All children must return SUCCESS
**Purpose**: Subscribe to required ROS topics and ensure data is available

**Children**:
- `PerceptionSubscriber` - Subscribe to `/perception/object_recognition/objects`
- `OdometrySubscriber` - Subscribe to `/localization/kinematic_state`

**Success**: All topics have received at least one message
**Failure**: Never fails, keeps RUNNING until data available

---

### 2. DetectObjects (Condition)

**Type**: `py_trees.behaviour.Behaviour` (Condition)
**Purpose**: Check if objects are detected from perception

**Corresponds to Autoware function**: `separateObjectsByPath()` (utils.cpp:2512)

**Logic**:
```python
def update(self):
    perception_objects = self.blackboard.get('perception_objects')

    if perception_objects is None:
        return py_trees.common.Status.RUNNING  # Waiting for data

    if len(perception_objects.objects) == 0:
        self.logger.info("No objects detected")
        return py_trees.common.Status.FAILURE  # No objects

    self.logger.info(f"Detected {len(perception_objects.objects)} objects")
    return py_trees.common.Status.SUCCESS  # Objects found
```

**Blackboard**:
- **READ**: `perception_objects` (PredictedObjects message)
- **WRITE**: None

---

### 3. FilterObjects (Action)

**Type**: `py_trees.behaviour.Behaviour` (Action)
**Purpose**: Filter objects by:
1. Valid object type (car, truck, bus - not pedestrian/bicycle)
2. Within detection range (5m - 150m)
3. On ego path (lateral distance check)

**Corresponds to Autoware functions**:
- `separateObjectsByPath()` (utils.cpp:2512) - Check if on path
- `filterTargetObjects()` (utils.cpp) - Filter by type/range

**Logic**:
```python
def update(self):
    raw_objects = self.blackboard.get('perception_objects')
    ego_pose = self.blackboard.get('ego_pose')

    filtered_objects = []

    for obj in raw_objects.objects:
        # Check 1: Valid type (car, truck, bus)
        if not self.is_valid_type(obj):
            continue

        # Check 2: Within detection range
        distance = self.calculate_distance(obj, ego_pose)
        if distance < 5.0 or distance > 150.0:
            continue

        # Check 3: On ego path (lateral distance)
        if not self.is_on_ego_path(obj, ego_pose):
            continue

        filtered_objects.append(obj)

    self.blackboard.set('filtered_objects', filtered_objects)
    self.logger.info(f"Filtered to {len(filtered_objects)} relevant objects")

    return py_trees.common.Status.SUCCESS
```

**Helper Functions**:

```python
def is_valid_type(self, obj) -> bool:
    """Check if object is car/truck/bus"""
    VALID_TYPES = [
        ObjectClassification.CAR,
        ObjectClassification.TRUCK,
        ObjectClassification.BUS,
        ObjectClassification.TRAILER
    ]

    if not obj.classification:
        return False

    # Get highest probability class
    highest = max(obj.classification, key=lambda c: c.probability)
    return highest.label in VALID_TYPES

def calculate_distance(self, obj, ego_pose) -> float:
    """Calculate Euclidean distance from ego to object"""
    obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
    ego_pos = ego_pose.position

    dx = obj_pos.x - ego_pos.x
    dy = obj_pos.y - ego_pos.y

    return math.sqrt(dx*dx + dy*dy)

def is_on_ego_path(self, obj, ego_pose) -> bool:
    """Simplified: Check if object is in front and lateral distance < lane_width/2"""
    obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
    ego_pos = ego_pose.position

    # Get ego heading
    ego_quat = ego_pose.orientation
    ego_yaw = self.quaternion_to_yaw(ego_quat)

    # Transform object to ego's coordinate frame
    dx = obj_pos.x - ego_pos.x
    dy = obj_pos.y - ego_pos.y

    # Rotate to ego frame
    cos_yaw = math.cos(-ego_yaw)
    sin_yaw = math.sin(-ego_yaw)

    x_ego_frame = dx * cos_yaw - dy * sin_yaw  # Longitudinal
    y_ego_frame = dx * sin_yaw + dy * cos_yaw  # Lateral

    # Check if in front (positive x)
    if x_ego_frame < 0:
        return False

    # Check lateral distance
    LANE_WIDTH = 3.5  # meters
    return abs(y_ego_frame) < (LANE_WIDTH / 2.0)

def quaternion_to_yaw(self, quat) -> float:
    """Convert quaternion to yaw angle"""
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)
```

**Blackboard**:
- **READ**: `perception_objects`, `ego_pose`
- **WRITE**: `filtered_objects` (list of objects on ego path)

---

### 4. ClassifyStoppedObjects (Action)

**Type**: `py_trees.behaviour.Behaviour` (Action)
**Purpose**: Classify which filtered objects are stopped (velocity < 1.0 m/s for > 2.0s)

**Corresponds to Autoware function**: `fillAvoidanceNecessity()` (utils.cpp)

**Logic**:
```python
def __init__(self, name: str, node: Node):
    super().__init__(name)
    self.node = node
    self.blackboard = self.attach_blackboard_client(name=name)

    # Parameters from Autoware config
    self.TH_MOVING_SPEED = 1.0  # m/s
    self.TH_MOVING_TIME = 2.0   # seconds

    # Track object history (to calculate stopped duration)
    self.object_history = {}  # {object_id: {'velocity': float, 'stopped_time': float}}

def update(self):
    filtered_objects = self.blackboard.get('filtered_objects')
    current_time = self.node.get_clock().now().nanoseconds / 1e9  # seconds

    stopped_objects = []

    for obj in filtered_objects:
        obj_id = self.get_object_id(obj)
        velocity = self.get_object_velocity(obj)

        # Initialize history if new object
        if obj_id not in self.object_history:
            self.object_history[obj_id] = {
                'first_seen': current_time,
                'stopped_since': None,
                'velocity': velocity
            }

        history = self.object_history[obj_id]

        # Check if object is stopped
        if velocity < self.TH_MOVING_SPEED:
            # Object is stopped
            if history['stopped_since'] is None:
                # Just started stopping
                history['stopped_since'] = current_time
                stopped_duration = 0.0
            else:
                # Been stopped for a while
                stopped_duration = current_time - history['stopped_since']

            # Check if stopped long enough
            if stopped_duration >= self.TH_MOVING_TIME:
                stopped_objects.append({
                    'object': obj,
                    'object_id': obj_id,
                    'velocity': velocity,
                    'stopped_duration': stopped_duration,
                    'distance': self.calculate_distance(obj)
                })
                self.logger.info(
                    f"Object {obj_id[:8]} stopped for {stopped_duration:.1f}s "
                    f"at {self.calculate_distance(obj):.1f}m"
                )
        else:
            # Object is moving, reset stopped time
            history['stopped_since'] = None

        history['velocity'] = velocity

    # Clean up old objects (not seen for 5 seconds)
    self.cleanup_old_objects(current_time, timeout=5.0)

    # Write to blackboard
    self.blackboard.set('stopped_objects', stopped_objects)
    self.blackboard.set('num_stopped_objects', len(stopped_objects))

    self.logger.info(
        f"Classified {len(stopped_objects)} stopped objects "
        f"out of {len(filtered_objects)} filtered objects"
    )

    return py_trees.common.Status.SUCCESS

def get_object_id(self, obj) -> str:
    """Get unique object ID"""
    # Convert UUID to string
    uuid_bytes = bytes(obj.object_id.uuid)
    return uuid_bytes.hex()

def get_object_velocity(self, obj) -> float:
    """Get object velocity magnitude"""
    vel = obj.kinematics.initial_twist_with_covariance.twist.linear
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

def calculate_distance(self, obj) -> float:
    """Calculate distance from ego to object"""
    ego_pose = self.blackboard.get('ego_pose')
    obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
    ego_pos = ego_pose.position

    dx = obj_pos.x - ego_pos.x
    dy = obj_pos.y - ego_pos.y

    return math.sqrt(dx*dx + dy*dy)

def cleanup_old_objects(self, current_time: float, timeout: float):
    """Remove objects not seen recently"""
    to_remove = []
    for obj_id, history in self.object_history.items():
        if current_time - history['first_seen'] > timeout:
            to_remove.append(obj_id)

    for obj_id in to_remove:
        del self.object_history[obj_id]
```

**Blackboard**:
- **READ**: `filtered_objects`, `ego_pose`
- **WRITE**: `stopped_objects` (list of objects that are stopped), `num_stopped_objects` (count)

---

## Complete BT Tree Code

```python
#!/usr/bin/env python3

import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry
import math

# ========== SUBSCRIBER NODES ==========

class PerceptionSubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to perception objects topic"""

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key='perception_objects', access=py_trees.common.Access.WRITE)
        self.latest_msg = None
        self.subscriber = None

    def setup(self, **kwargs):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscriber = self.node.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self._callback,
            qos
        )

    def update(self) -> py_trees.common.Status:
        if self.latest_msg is not None:
            self.blackboard.set('perception_objects', self.latest_msg)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def _callback(self, msg: PredictedObjects) -> None:
        self.latest_msg = msg


class OdometrySubscriber(py_trees.behaviour.Behaviour):
    """Subscribe to odometry topic"""

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key='odometry', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='ego_pose', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='ego_velocity', access=py_trees.common.Access.WRITE)
        self.latest_msg = None
        self.subscriber = None

    def setup(self, **kwargs):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscriber = self.node.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self._callback,
            qos
        )

    def update(self) -> py_trees.common.Status:
        if self.latest_msg is not None:
            self.blackboard.set('odometry', self.latest_msg)
            self.blackboard.set('ego_pose', self.latest_msg.pose.pose)

            # Extract velocity magnitude
            vel = self.latest_msg.twist.twist.linear
            velocity = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            self.blackboard.set('ego_velocity', velocity)

            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def _callback(self, msg: Odometry) -> None:
        self.latest_msg = msg


# ========== DETECTION & CLASSIFICATION NODES ==========

class DetectObjects(py_trees.behaviour.Behaviour):
    """Check if objects are detected"""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key='perception_objects', access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        perception_objects = self.blackboard.get('perception_objects')

        if perception_objects is None:
            self.logger.info("⏳ Waiting for perception data...")
            return py_trees.common.Status.RUNNING

        num_objects = len(perception_objects.objects)

        if num_objects == 0:
            self.logger.info("✗ No objects detected")
            return py_trees.common.Status.FAILURE

        self.logger.info(f"✓ Detected {num_objects} objects")
        return py_trees.common.Status.SUCCESS


class FilterObjects(py_trees.behaviour.Behaviour):
    """Filter objects by type, range, and path"""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key='perception_objects', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='ego_pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='filtered_objects', access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        from autoware_perception_msgs.msg import ObjectClassification

        raw_objects = self.blackboard.get('perception_objects')
        ego_pose = self.blackboard.get('ego_pose')

        if raw_objects is None or ego_pose is None:
            return py_trees.common.Status.FAILURE

        filtered_objects = []

        for obj in raw_objects.objects:
            # Check 1: Valid type
            if not self.is_valid_type(obj):
                continue

            # Check 2: Within detection range
            distance = self.calculate_distance(obj, ego_pose)
            if distance < 5.0 or distance > 150.0:
                continue

            # Check 3: On ego path
            if not self.is_on_ego_path(obj, ego_pose):
                continue

            filtered_objects.append(obj)

        self.blackboard.set('filtered_objects', filtered_objects)
        self.logger.info(f"✓ Filtered to {len(filtered_objects)}/{len(raw_objects.objects)} objects")

        return py_trees.common.Status.SUCCESS

    def is_valid_type(self, obj) -> bool:
        from autoware_perception_msgs.msg import ObjectClassification

        VALID_TYPES = [
            ObjectClassification.CAR,
            ObjectClassification.TRUCK,
            ObjectClassification.BUS,
            ObjectClassification.TRAILER
        ]

        if not obj.classification:
            return False

        highest = max(obj.classification, key=lambda c: c.probability)
        return highest.label in VALID_TYPES

    def calculate_distance(self, obj, ego_pose) -> float:
        obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
        ego_pos = ego_pose.position

        dx = obj_pos.x - ego_pos.x
        dy = obj_pos.y - ego_pos.y

        return math.sqrt(dx*dx + dy*dy)

    def is_on_ego_path(self, obj, ego_pose) -> bool:
        obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
        ego_pos = ego_pose.position

        ego_quat = ego_pose.orientation
        ego_yaw = self.quaternion_to_yaw(ego_quat)

        dx = obj_pos.x - ego_pos.x
        dy = obj_pos.y - ego_pos.y

        cos_yaw = math.cos(-ego_yaw)
        sin_yaw = math.sin(-ego_yaw)

        x_ego_frame = dx * cos_yaw - dy * sin_yaw
        y_ego_frame = dx * sin_yaw + dy * cos_yaw

        if x_ego_frame < 0:
            return False

        LANE_WIDTH = 3.5
        return abs(y_ego_frame) < (LANE_WIDTH / 2.0)

    def quaternion_to_yaw(self, quat) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


class ClassifyStoppedObjects(py_trees.behaviour.Behaviour):
    """Classify which objects are stopped"""

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key='filtered_objects', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='ego_pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='stopped_objects', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='num_stopped_objects', access=py_trees.common.Access.WRITE)

        self.TH_MOVING_SPEED = 1.0
        self.TH_MOVING_TIME = 2.0
        self.object_history = {}

    def update(self) -> py_trees.common.Status:
        filtered_objects = self.blackboard.get('filtered_objects')
        current_time = self.node.get_clock().now().nanoseconds / 1e9

        if filtered_objects is None:
            return py_trees.common.Status.FAILURE

        stopped_objects = []

        for obj in filtered_objects:
            obj_id = self.get_object_id(obj)
            velocity = self.get_object_velocity(obj)

            if obj_id not in self.object_history:
                self.object_history[obj_id] = {
                    'first_seen': current_time,
                    'stopped_since': None,
                    'velocity': velocity
                }

            history = self.object_history[obj_id]

            if velocity < self.TH_MOVING_SPEED:
                if history['stopped_since'] is None:
                    history['stopped_since'] = current_time
                    stopped_duration = 0.0
                else:
                    stopped_duration = current_time - history['stopped_since']

                if stopped_duration >= self.TH_MOVING_TIME:
                    stopped_objects.append({
                        'object': obj,
                        'object_id': obj_id,
                        'velocity': velocity,
                        'stopped_duration': stopped_duration,
                        'distance': self.calculate_distance(obj)
                    })
            else:
                history['stopped_since'] = None

            history['velocity'] = velocity

        self.cleanup_old_objects(current_time, timeout=5.0)

        self.blackboard.set('stopped_objects', stopped_objects)
        self.blackboard.set('num_stopped_objects', len(stopped_objects))

        self.logger.info(
            f"✓ Classified {len(stopped_objects)} stopped objects "
            f"out of {len(filtered_objects)} filtered"
        )

        return py_trees.common.Status.SUCCESS

    def get_object_id(self, obj) -> str:
        uuid_bytes = bytes(obj.object_id.uuid)
        return uuid_bytes.hex()

    def get_object_velocity(self, obj) -> float:
        vel = obj.kinematics.initial_twist_with_covariance.twist.linear
        return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

    def calculate_distance(self, obj) -> float:
        ego_pose = self.blackboard.get('ego_pose')
        obj_pos = obj.kinematics.initial_pose_with_covariance.pose.position
        ego_pos = ego_pose.position

        dx = obj_pos.x - ego_pos.x
        dy = obj_pos.y - ego_pos.y

        return math.sqrt(dx*dx + dy*dy)

    def cleanup_old_objects(self, current_time: float, timeout: float):
        to_remove = []
        for obj_id, history in self.object_history.items():
            if current_time - history['first_seen'] > timeout:
                to_remove.append(obj_id)

        for obj_id in to_remove:
            del self.object_history[obj_id]


# ========== MAIN BT MANAGER ==========

class BTDetectionNode(Node):
    """BT Manager for Object Detection & Classification"""

    def __init__(self):
        super().__init__('bt_detection_node')

        self.tree = self.create_detection_tree()

        frequency = 10.0  # Hz
        self.timer = self.create_timer(1.0 / frequency, self.tick_callback)

    def create_detection_tree(self):
        """Build the detection & classification BT"""

        # Root sequence
        root = py_trees.composites.Sequence(name="Root", memory=True)

        # 1. Subscribe to topics (parallel)
        subscribe_topics = py_trees.composites.Parallel(
            name="SubscribeTopics",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        perception_sub = PerceptionSubscriber(name="PerceptionSubscriber", node=self)
        odometry_sub = OdometrySubscriber(name="OdometrySubscriber", node=self)
        subscribe_topics.add_children([perception_sub, odometry_sub])

        # 2. Detect objects
        detect_objects = DetectObjects(name="DetectObjects")

        # 3. Filter objects
        filter_objects = FilterObjects(name="FilterObjects")

        # 4. Classify stopped objects
        classify_stopped = ClassifyStoppedObjects(name="ClassifyStoppedObjects", node=self)

        # Build tree
        root.add_children([
            subscribe_topics,
            detect_objects,
            filter_objects,
            classify_stopped
        ])

        # Create BT
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)

        # Add snapshot visitor for visualization
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        tree.visitors.append(snapshot_visitor)
        tree.add_post_tick_handler(lambda tree: self.post_tick_handler(snapshot_visitor, tree))

        tree.setup(timeout=15.0, node=self)
        return tree

    def post_tick_handler(self, snapshot_visitor, behaviour_tree):
        """Print tree after each tick"""
        print("\n" + "="*50)
        print(py_trees.display.unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        ))
        print("="*50)

    def tick_callback(self):
        """Tick the tree at 10 Hz"""
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BTDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## How to Run

### 1. Create Package Structure

```bash
cd ~/your_workspace/src
ros2 pkg create bt_obstacle_avoidance --build-type ament_python --dependencies rclpy py_trees py_trees_ros autoware_perception_msgs nav_msgs
```

### 2. Save Code

Save the complete code above to:
```
~/your_workspace/src/bt_obstacle_avoidance/bt_obstacle_avoidance/bt_detection_node.py
```

### 3. Update `setup.py`

```python
entry_points={
    'console_scripts': [
        'bt_detection_node = bt_obstacle_avoidance.bt_detection_node:main',
    ],
},
```

### 4. Build

```bash
cd ~/your_workspace
colcon build --packages-select bt_obstacle_avoidance
source install/setup.bash
```

### 5. Run

**Terminal 1**: Start Autoware (with perception)
```bash
cd ~/autoware
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml ...
```

**Terminal 2**: Run BT Detection Node
```bash
source ~/your_workspace/install/setup.bash
ros2 run bt_obstacle_avoidance bt_detection_node
```

---

## Expected Output

```
==================================================
Root [SUCCESS]
  SubscribeTopics [SUCCESS]
    PerceptionSubscriber [SUCCESS]
    OdometrySubscriber [SUCCESS]
  DetectObjects [SUCCESS]
  FilterObjects [SUCCESS]
  ClassifyStoppedObjects [SUCCESS]
==================================================

[bt_detection_node]: ✓ Detected 5 objects
[bt_detection_node]: ✓ Filtered to 2/5 objects
[bt_detection_node]: ✓ Classified 1 stopped objects out of 2 filtered
```

---

## Blackboard Data Flow

| Node | Reads | Writes |
|------|-------|--------|
| **PerceptionSubscriber** | - | `perception_objects` |
| **OdometrySubscriber** | - | `odometry`, `ego_pose`, `ego_velocity` |
| **DetectObjects** | `perception_objects` | - |
| **FilterObjects** | `perception_objects`, `ego_pose` | `filtered_objects` |
| **ClassifyStoppedObjects** | `filtered_objects`, `ego_pose` | `stopped_objects`, `num_stopped_objects` |

---

## Next Steps

After this works, you can add:
1. **Stop Decision** - Check if need to stop for stopped objects
2. **Safety Check** - Validate if safe to stop
3. **Execute Stop** - Publish stop command

---

**Created**: 2025-10-03
**For**: Thesis BT Implementation - Object Detection & Classification Phase

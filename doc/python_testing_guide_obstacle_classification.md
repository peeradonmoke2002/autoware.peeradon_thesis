# Python Testing Guide: Obstacle Classification for Static Obstacle Avoidance

**Purpose:** Create simple Python scripts to test object classification logic for broken-down vehicle detection
**Target Module:** `autoware_behavior_path_static_obstacle_avoidance_module`
**Author:** BT Implementation Testing Guide
**Date:** 2025-10-02

---

## 1. Overview

This guide helps you create Python test scripts to understand and test the obstacle classification logic used in Autoware's static obstacle avoidance module. This is the **first step** before implementing BT-based replacements.

### Testing Approach:
1. **Subscribe to perception topics** to get object data
2. **Apply classification rules** (same as current FSM)
3. **Publish debug output** to visualize classification results
4. **Compare with actual Autoware behavior** to validate understanding

---

## 2. Required ROS 2 Topics for Testing

### 2.1 Input Topics (Subscribe)

**Mandatory Inputs:**

| Topic Name | Message Type | Purpose | Frequency |
|------------|-------------|---------|-----------|
| `/perception/object_recognition/objects` | `autoware_perception_msgs/msg/PredictedObjects` | Dynamic objects from perception | ~10 Hz |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | Ego vehicle position & velocity | ~50 Hz |
| `/map/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | HD Map (lanelet2 format) | Once at startup |
| `/planning/mission_planning/route` | `autoware_planning_msgs/msg/LaneletRoute` | Current route from mission planner | When route changes |

**Optional (for advanced testing):**

| Topic Name | Message Type | Purpose |
|------------|-------------|---------|
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform tree (coordinate frames) |
| `/perception/obstacle_segmentation/pointcloud` | `sensor_msgs/msg/PointCloud2` | Raw point cloud data |

### 2.2 Output Topics (Publish for Debugging)

| Topic Name | Message Type | Purpose |
|------------|-------------|---------|
| `/test/classified_objects` | `visualization_msgs/msg/MarkerArray` | Visualize classified objects in RViz |
| `/test/avoidance_debug` | `std_msgs/msg/String` | Text debug messages |
| `/test/object_classification_result` | Custom message (see below) | Classification results per object |

---

## 3. Object Classification Logic (Python Implementation)

### 3.1 Classification Rules (from `static_obstacle_avoidance.param.yaml`)

```python
# Classification parameters (from Autoware config)
CLASSIFICATION_PARAMS = {
    'car': {
        'th_moving_speed': 1.0,      # m/s - slower than this = stopped
        'th_moving_time': 2.0,        # seconds - must be stopped for this long
        'lateral_margin_soft': 0.5,   # m - preferred safety margin
        'lateral_margin_hard': 0.2,   # m - minimum safety margin
    },
    'truck': {
        'th_moving_speed': 1.0,
        'th_moving_time': 2.0,
        'lateral_margin_soft': 0.5,
        'lateral_margin_hard': 0.2,
    },
    'bus': {
        'th_moving_speed': 1.0,
        'th_moving_time': 2.0,
        'lateral_margin_soft': 0.5,
        'lateral_margin_hard': 0.2,
    },
}

# Target filtering parameters
FILTERING_PARAMS = {
    'object_check_goal_distance': 20.0,          # m - ignore objects closer than this to goal
    'th_offset_from_centerline': 1.0,            # m - parked vehicle detection threshold
    'th_shiftable_ratio': 0.8,                   # road shoulder width ratio
    'min_forward_distance': 50.0,                # m - detection range start
    'max_forward_distance': 150.0,               # m - detection range end
    'backward_distance': 10.0,                   # m - detection range behind ego
}
```

### 3.2 Step-by-Step Classification Algorithm

```python
import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
import math
from collections import defaultdict
import time

class ObjectClassifierNode(Node):
    """
    Simple object classifier for testing static obstacle avoidance logic.
    """

    def __init__(self):
        super().__init__('object_classifier_test')

        # Subscribers
        self.objects_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.objects_callback,
            10
        )

        self.odometry_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/test/classified_objects',
            10
        )

        # State tracking
        self.ego_pose = None
        self.ego_velocity = None
        self.object_history = defaultdict(list)  # Track object velocities over time

        self.get_logger().info('Object Classifier Test Node Started')

    def odometry_callback(self, msg):
        """Store ego vehicle state."""
        self.ego_pose = msg.pose.pose
        self.ego_velocity = msg.twist.twist.linear.x

    def objects_callback(self, msg):
        """
        Main classification logic - called when new objects arrive.
        """
        if self.ego_pose is None:
            self.get_logger().warn('Waiting for ego pose...')
            return

        classified_objects = []
        current_time = time.time()

        for obj in msg.objects:
            # Step 1: Object Type Check
            if not self.is_target_type(obj):
                continue

            # Step 2: Distance Check
            distance = self.calculate_distance_to_ego(obj)
            if not self.is_within_detection_range(distance):
                continue

            # Step 3: Movement Check (requires tracking over time)
            classification = self.classify_movement(obj, current_time)

            # Step 4: Position Check
            offset_from_centerline = self.calculate_offset_from_centerline(obj)
            is_on_road_edge = self.is_on_road_edge(offset_from_centerline)

            # Final classification
            result = {
                'object': obj,
                'distance': distance,
                'velocity': obj.kinematics.initial_twist_with_covariance.twist.linear.x,
                'classification': classification,
                'offset_from_centerline': offset_from_centerline,
                'is_avoidable': classification == 'STOPPED' and is_on_road_edge,
                'reason': self.get_classification_reason(obj, classification, is_on_road_edge)
            }

            classified_objects.append(result)

        # Publish visualization
        self.publish_markers(classified_objects)

        # Print results
        self.print_classification_results(classified_objects)

    def is_target_type(self, obj):
        """
        Check if object is a target type (CAR, TRUCK, BUS).
        ObjectClassification values:
        - UNKNOWN = 0
        - CAR = 1
        - TRUCK = 2
        - BUS = 3
        - TRAILER = 4
        - MOTORCYCLE = 5
        - BICYCLE = 6
        - PEDESTRIAN = 7
        """
        target_types = [1, 2, 3]  # CAR, TRUCK, BUS
        if obj.classification:
            return obj.classification[0].label in target_types
        return False

    def calculate_distance_to_ego(self, obj):
        """Calculate Euclidean distance from ego to object."""
        dx = obj.kinematics.initial_pose_with_covariance.pose.position.x - self.ego_pose.position.x
        dy = obj.kinematics.initial_pose_with_covariance.pose.position.y - self.ego_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def is_within_detection_range(self, distance):
        """Check if object is within detection area."""
        return (FILTERING_PARAMS['min_forward_distance'] <= distance <=
                FILTERING_PARAMS['max_forward_distance'])

    def classify_movement(self, obj, current_time):
        """
        Classify object as MOVING or STOPPED based on velocity history.

        Rules:
        - If velocity < 1.0 m/s for > 2.0 seconds → STOPPED
        - Otherwise → MOVING
        """
        obj_id = self.get_object_id(obj)
        velocity = obj.kinematics.initial_twist_with_covariance.twist.linear.x

        # Store velocity history
        self.object_history[obj_id].append({
            'time': current_time,
            'velocity': velocity
        })

        # Keep only last 3 seconds of history
        self.object_history[obj_id] = [
            entry for entry in self.object_history[obj_id]
            if current_time - entry['time'] < 3.0
        ]

        # Check if stopped for required duration
        history = self.object_history[obj_id]
        if not history:
            return 'UNKNOWN'

        # Check if consistently below threshold
        stopped_duration = 0
        for i in range(len(history) - 1, -1, -1):
            if history[i]['velocity'] < CLASSIFICATION_PARAMS['car']['th_moving_speed']:
                stopped_duration = current_time - history[i]['time']
            else:
                break

        if stopped_duration >= CLASSIFICATION_PARAMS['car']['th_moving_time']:
            return 'STOPPED'
        else:
            return 'MOVING'

    def get_object_id(self, obj):
        """Get unique object ID (use UUID if available)."""
        # For simplicity, use position hash
        pos = obj.kinematics.initial_pose_with_covariance.pose.position
        return f"{int(pos.x * 10)}_{int(pos.y * 10)}"

    def calculate_offset_from_centerline(self, obj):
        """
        Calculate lateral offset from lane centerline.

        Note: This is simplified. Real implementation needs lanelet2 map.
        For testing, we approximate using road coordinate system.
        """
        # Simplified: assume ego is on centerline, calculate perpendicular distance
        # In real implementation, use lanelet2 to find nearest centerline point

        # TODO: Implement proper lanelet2 integration
        # For now, return a mock value
        return 0.8  # meters (example value)

    def is_on_road_edge(self, offset):
        """
        Check if object is on road edge (parked/broken down).

        Object is considered on edge if:
        - Offset from centerline > threshold (1.0m)
        """
        return offset > FILTERING_PARAMS['th_offset_from_centerline']

    def get_classification_reason(self, obj, movement_class, is_on_edge):
        """
        Return reason for classification decision.
        Maps to ObjectInfo enum from data_structs.hpp
        """
        if movement_class == 'MOVING':
            return 'MOVING_OBJECT'

        if not is_on_edge:
            return 'TOO_NEAR_TO_CENTERLINE'

        if movement_class == 'STOPPED' and is_on_edge:
            return 'AVOIDABLE'

        return 'UNKNOWN'

    def publish_markers(self, classified_objects):
        """
        Publish RViz markers for visualization.

        Color coding:
        - GREEN: Avoidable (stopped, on edge)
        - RED: Not avoidable (moving or too close to centerline)
        - YELLOW: Unknown/under observation
        """
        marker_array = MarkerArray()

        for i, result in enumerate(classified_objects):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "classified_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            obj_pose = result['object'].kinematics.initial_pose_with_covariance.pose
            marker.pose = obj_pose

            # Size (use object shape if available)
            marker.scale.x = 4.5  # car length
            marker.scale.y = 1.8  # car width
            marker.scale.z = 1.5  # car height

            # Color based on classification
            if result['is_avoidable']:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            elif result['classification'] == 'MOVING':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = obj_pose
            text_marker.pose.position.z += 2.0  # Above object
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{result['classification']}\n{result['reason']}"

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def print_classification_results(self, results):
        """Print classification results to console."""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Classified {len(results)} objects:')
        for i, result in enumerate(results):
            self.get_logger().info(
                f"  [{i}] Distance: {result['distance']:.2f}m, "
                f"Vel: {result['velocity']:.2f}m/s, "
                f"Class: {result['classification']}, "
                f"Avoidable: {result['is_avoidable']}, "
                f"Reason: {result['reason']}"
            )
        self.get_logger().info('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectClassifierNode()

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

## 4. How to Run the Test Script

### 4.1 Setup Python ROS 2 Environment

```bash
# Navigate to your workspace
cd ~/autoware

# Source ROS 2 and Autoware
source install/setup.bash

# Install Python dependencies (if needed)
pip3 install transforms3d  # For coordinate transformations
```

### 4.2 Create Test Package Structure

```bash
# Create a test package
cd ~/autoware/src
mkdir -p test_packages/obstacle_classification_test
cd test_packages/obstacle_classification_test

# Create package structure
mkdir -p obstacle_classification_test
touch obstacle_classification_test/__init__.py
touch obstacle_classification_test/classifier_node.py
touch setup.py
touch package.xml
```

### 4.3 Create `setup.py`

```python
from setuptools import setup

package_name = 'obstacle_classification_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Object classification test for BT implementation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'classifier_node = obstacle_classification_test.classifier_node:main',
        ],
    },
)
```

### 4.4 Create `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>obstacle_classification_test</name>
  <version>0.0.1</version>
  <description>Object classification test for BT implementation</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>autoware_perception_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 4.5 Build and Run

```bash
# Build the package
cd ~/autoware
colcon build --packages-select obstacle_classification_test
source install/setup.bash

# Run the classifier node
ros2 run obstacle_classification_test classifier_node
```

### 4.6 Visualize in RViz

```bash
# In another terminal
rviz2

# Add visualization:
# 1. Set Fixed Frame to "map"
# 2. Add -> By topic -> /test/classified_objects -> MarkerArray
```

---

## 5. Testing Scenarios

### 5.1 Scenario 1: Stationary Broken-Down Vehicle

**Setup:**
- Place a static vehicle object in front of ego
- Object velocity = 0 m/s
- Position: 30m ahead, offset 1.2m from centerline (on road edge)

**Expected Classification:**
- After 2 seconds: `STOPPED`
- Reason: `AVOIDABLE`
- Color in RViz: GREEN

### 5.2 Scenario 2: Slow Moving Vehicle

**Setup:**
- Vehicle moving at 0.5 m/s (below 1.0 threshold)
- Position: 50m ahead, centerline

**Expected Classification:**
- Classification: `STOPPED` (after 2s)
- Reason: `TOO_NEAR_TO_CENTERLINE`
- Color: RED (not avoidable due to position)

### 5.3 Scenario 3: Normal Traffic

**Setup:**
- Vehicle moving at 15 m/s
- Position: 80m ahead

**Expected Classification:**
- Classification: `MOVING`
- Reason: `MOVING_OBJECT`
- Color: RED

---

## 6. Next Steps: Integrate with Lanelet2 Map

The current implementation uses a simplified centerline calculation. For production testing:

### 6.1 Add Lanelet2 Integration

```python
import lanelet2
from autoware_map_msgs.msg import LaneletMapBin

class ObjectClassifierNode(Node):
    def __init__(self):
        # ... existing code ...

        # Add map subscriber
        self.map_sub = self.create_subscription(
            LaneletMapBin,
            '/map/vector_map',
            self.map_callback,
            1
        )
        self.lanelet_map = None

    def map_callback(self, msg):
        """Load lanelet2 map."""
        # Convert ROS message to lanelet2 map
        # This requires lanelet2 Python bindings
        # TODO: Implement lanelet2 parsing
        pass

    def calculate_offset_from_centerline(self, obj):
        """Calculate actual offset using lanelet2 map."""
        if self.lanelet_map is None:
            return 0.8  # fallback

        # 1. Find nearest lanelet to object
        # 2. Get centerline of lanelet
        # 3. Calculate perpendicular distance
        # TODO: Implement with lanelet2 API
        pass
```

### 6.2 Validate Against Real Autoware

Run your classifier alongside real Autoware and compare:

```bash
# Terminal 1: Run Autoware (with static obstacle avoidance)
ros2 launch autoware_launch planning.launch.xml

# Terminal 2: Run your classifier
ros2 run obstacle_classification_test classifier_node

# Terminal 3: Monitor both outputs
ros2 topic echo /planning/debug/avoidance_debug_message_array
ros2 topic echo /test/classified_objects
```

---

## 7. Summary

**What You've Built:**
- ✅ Object classifier that mimics Autoware's logic
- ✅ RViz visualization for debugging
- ✅ Test framework for classification rules

**What This Enables:**
- Test BT condition nodes before full integration
- Validate understanding of FSM logic
- Create ground truth data for BT testing
- Debug classification issues independently

**Next Step:**
After validating classification logic, you can:
1. Convert classification rules to BT condition nodes
2. Implement path planning logic (shift line generation)
3. Integrate with Autoware's planning framework

---

**End of Guide**

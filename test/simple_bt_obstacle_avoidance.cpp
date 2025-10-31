/**
 * Simple Behavior Tree Example for Obstacle Avoidance
 *
 * Purpose: Test if BT works for broken-down vehicle detection
 * This is a simplified version to understand BT concepts in C++
 *
 * Python vs C++ Concepts:
 * - Python class -> C++ class (similar, but with types and pointers)
 * - Python self -> C++ this (pointer to current object)
 * - Python dict -> C++ std::map or Blackboard (key-value storage)
 * - Python None -> C++ nullptr (null pointer)
 */

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <cmath>
#include <vector>

// ============================================================================
// DATA STRUCTURES (Like Python dataclasses)
// ============================================================================

/**
 * Python equivalent:
 * @dataclass
 * class Object:
 *     id: str
 *     velocity: float
 *     distance: float
 *     stopped_duration: float
 */
struct Object {
    std::string id;          // Like Python str
    double velocity;         // Like Python float
    double distance;         // Like Python float
    double stopped_duration; // Like Python float

    // Constructor (like Python __init__)
    Object(std::string id, double vel, double dist, double stopped_dur)
        : id(id), velocity(vel), distance(dist), stopped_duration(stopped_dur) {}
};

/**
 * Python equivalent:
 * perception_objects = [Object(...), Object(...), ...]
 */
using ObjectList = std::vector<Object>;  // Like Python List[Object]


// ============================================================================
// BT CONDITION NODES (Like py_trees.behaviour.Behaviour returning SUCCESS/FAILURE)
// ============================================================================

/**
 * DetectObjects: Check if any objects are detected
 *
 * Python equivalent:
 * class DetectObjects(py_trees.behaviour.Behaviour):
 *     def update(self):
 *         objects = self.blackboard.get('perception_objects')
 *         if objects is None or len(objects) == 0:
 *             return py_trees.common.Status.FAILURE
 *         return py_trees.common.Status.SUCCESS
 */
class DetectObjects : public BT::ConditionNode
{
public:
    // Constructor - like Python __init__(self, name)
    DetectObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    // Required method to specify input/output ports (blackboard keys)
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<ObjectList>("perception_objects") };
    }

    // The main logic - like Python update() method
    BT::NodeStatus tick() override
    {
        // Get objects from blackboard (like self.blackboard.get())
        auto objects_result = getInput<ObjectList>("perception_objects");

        // Check if data exists (like checking if None in Python)
        if (!objects_result) {
            std::cout << "[DetectObjects] ⏳ No perception data available\n";
            return BT::NodeStatus::FAILURE;
        }

        ObjectList objects = objects_result.value();

        if (objects.empty()) {
            std::cout << "[DetectObjects] ✗ No objects detected\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[DetectObjects] ✓ Detected " << objects.size() << " objects\n";
        return BT::NodeStatus::SUCCESS;
    }
};


// ============================================================================
// BT ACTION NODES (Like py_trees.behaviour.Behaviour that does work)
// ============================================================================

/**
 * FilterObjects: Filter objects by velocity and distance
 *
 * Python equivalent:
 * class FilterObjects(py_trees.behaviour.Behaviour):
 *     def update(self):
 *         raw_objects = self.blackboard.get('perception_objects')
 *         filtered = [obj for obj in raw_objects if self.is_valid(obj)]
 *         self.blackboard.set('filtered_objects', filtered)
 *         return py_trees.common.Status.SUCCESS
 */
class FilterObjects : public BT::SyncActionNode
{
public:
    FilterObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<ObjectList>("perception_objects"),   // Read from blackboard
            BT::OutputPort<ObjectList>("filtered_objects")     // Write to blackboard
        };
    }

    BT::NodeStatus tick() override
    {
        // Get input from blackboard
        auto objects_result = getInput<ObjectList>("perception_objects");
        if (!objects_result) {
            return BT::NodeStatus::FAILURE;
        }

        ObjectList objects = objects_result.value();
        ObjectList filtered_objects;

        // Filter logic (like Python list comprehension)
        // Python: filtered = [obj for obj in objects if 5.0 < obj.distance < 150.0]
        for (const auto& obj : objects) {  // Like Python: for obj in objects:
            // Check distance range
            if (obj.distance >= 5.0 && obj.distance <= 150.0) {
                filtered_objects.push_back(obj);  // Like Python: filtered.append(obj)
            }
        }

        std::cout << "[FilterObjects] ✓ Filtered to " << filtered_objects.size()
                  << "/" << objects.size() << " objects\n";

        // Write output to blackboard (like self.blackboard.set())
        setOutput("filtered_objects", filtered_objects);

        return BT::NodeStatus::SUCCESS;
    }
};


/**
 * ClassifyStoppedObjects: Classify which objects are stopped
 *
 * Python equivalent:
 * class ClassifyStoppedObjects(py_trees.behaviour.Behaviour):
 *     def update(self):
 *         filtered_objects = self.blackboard.get('filtered_objects')
 *         stopped = [obj for obj in filtered_objects
 *                   if obj.velocity < 1.0 and obj.stopped_duration >= 2.0]
 *         self.blackboard.set('stopped_objects', stopped)
 *         return py_trees.common.Status.SUCCESS
 */
class ClassifyStoppedObjects : public BT::SyncActionNode
{
private:
    // Constants (like Python class variables)
    const double MOVING_SPEED_THRESHOLD = 1.0;  // m/s
    const double MOVING_TIME_THRESHOLD = 2.0;   // seconds

public:
    ClassifyStoppedObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<ObjectList>("filtered_objects"),
            BT::OutputPort<ObjectList>("stopped_objects"),
            BT::OutputPort<int>("num_stopped_objects")
        };
    }

    BT::NodeStatus tick() override
    {
        auto objects_result = getInput<ObjectList>("filtered_objects");
        if (!objects_result) {
            return BT::NodeStatus::FAILURE;
        }

        ObjectList objects = objects_result.value();
        ObjectList stopped_objects;

        // Classify stopped objects
        for (const auto& obj : objects) {
            if (obj.velocity < MOVING_SPEED_THRESHOLD &&
                obj.stopped_duration >= MOVING_TIME_THRESHOLD) {
                stopped_objects.push_back(obj);

                std::cout << "[ClassifyStoppedObjects] Object " << obj.id
                         << " stopped for " << obj.stopped_duration << "s "
                         << "at " << obj.distance << "m\n";
            }
        }

        std::cout << "[ClassifyStoppedObjects] ✓ Classified "
                  << stopped_objects.size() << " stopped objects "
                  << "out of " << objects.size() << " filtered\n";

        // Write outputs to blackboard
        setOutput("stopped_objects", stopped_objects);
        setOutput("num_stopped_objects", static_cast<int>(stopped_objects.size()));

        return BT::NodeStatus::SUCCESS;
    }
};


// ============================================================================
// MAIN FUNCTION - CREATE AND RUN BT
// ============================================================================

int main()
{
    std::cout << "========================================\n";
    std::cout << "Simple BT Obstacle Avoidance Test\n";
    std::cout << "========================================\n\n";

    // ========================================================================
    // STEP 1: Create BT Factory (like py_trees factory)
    // ========================================================================
    BT::BehaviorTreeFactory factory;

    // Register our custom nodes (like registering behaviours in py_trees)
    factory.registerNodeType<DetectObjects>("DetectObjects");
    factory.registerNodeType<FilterObjects>("FilterObjects");
    factory.registerNodeType<ClassifyStoppedObjects>("ClassifyStoppedObjects");


    // ========================================================================
    // STEP 2: Define BT Structure in XML (like py_trees tree structure)
    // ========================================================================
    /**
     * Python equivalent:
     * root = py_trees.composites.Sequence(name="Root")
     * root.add_children([
     *     DetectObjects("DetectObjects"),
     *     FilterObjects("FilterObjects"),
     *     ClassifyStoppedObjects("ClassifyStoppedObjects")
     * ])
     */
    static const char* xml_text = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="ObstacleAvoidance">
                <Sequence name="Root">
                    <DetectObjects
                        perception_objects="{perception_objects}" />
                    <FilterObjects
                        perception_objects="{perception_objects}"
                        filtered_objects="{filtered_objects}" />
                    <ClassifyStoppedObjects
                        filtered_objects="{filtered_objects}"
                        stopped_objects="{stopped_objects}"
                        num_stopped_objects="{num_stopped_objects}" />
                </Sequence>
            </BehaviorTree>
        </root>
    )";


    // ========================================================================
    // STEP 3: Create BT from XML
    // ========================================================================
    auto tree = factory.createTreeFromText(xml_text);


    // ========================================================================
    // STEP 4: Create Test Data (Simulated sensor data)
    // ========================================================================
    /**
     * Python equivalent:
     * perception_objects = [
     *     Object(id="obj_1", velocity=15.0, distance=50.0, stopped_duration=0.0),
     *     Object(id="obj_2", velocity=0.5, distance=30.0, stopped_duration=3.5),
     *     ...
     * ]
     */
    ObjectList perception_objects = {
        Object("obj_1", 15.0, 50.0, 0.0),    // Moving car at 50m
        Object("obj_2", 0.5, 30.0, 3.5),     // Stopped car at 30m (broken-down!)
        Object("obj_3", 0.2, 120.0, 5.0),    // Stopped car at 120m (broken-down!)
        Object("obj_4", 20.0, 200.0, 0.0),   // Moving car at 200m (too far)
        Object("obj_5", 0.8, 3.0, 1.5),      // Stopped car at 3m (too close, stopped < 2s)
    };

    std::cout << "Test Data: " << perception_objects.size() << " objects\n";
    std::cout << "- obj_1: Moving car (15.0 m/s) at 50m\n";
    std::cout << "- obj_2: Stopped car (0.5 m/s, 3.5s) at 30m ← BROKEN-DOWN\n";
    std::cout << "- obj_3: Stopped car (0.2 m/s, 5.0s) at 120m ← BROKEN-DOWN\n";
    std::cout << "- obj_4: Moving car (20.0 m/s) at 200m (too far)\n";
    std::cout << "- obj_5: Stopped car (0.8 m/s, 1.5s) at 3m (too close)\n\n";


    // ========================================================================
    // STEP 5: Set initial blackboard data (like py_trees blackboard)
    // ========================================================================
    /**
     * Python equivalent:
     * blackboard = py_trees.blackboard.Client()
     * blackboard.set('perception_objects', perception_objects)
     */
    tree.rootBlackboard()->set("perception_objects", perception_objects);


    // ========================================================================
    // STEP 6: Tick the tree (like tree.tick() in py_trees)
    // ========================================================================
    std::cout << "Ticking BT...\n";
    std::cout << "========================================\n\n";

    BT::NodeStatus status = tree.tickOnce();  // Like tree.tick() in Python

    std::cout << "\n========================================\n";
    std::cout << "BT Status: ";

    // Print result (like checking tree status in py_trees)
    switch (status) {
        case BT::NodeStatus::SUCCESS:
            std::cout << "SUCCESS ✓\n";
            break;
        case BT::NodeStatus::FAILURE:
            std::cout << "FAILURE ✗\n";
            break;
        case BT::NodeStatus::RUNNING:
            std::cout << "RUNNING ⏳\n";
            break;
        default:
            std::cout << "IDLE\n";
    }


    // ========================================================================
    // STEP 7: Read results from blackboard
    // ========================================================================
    /**
     * Python equivalent:
     * stopped_objects = blackboard.get('stopped_objects')
     * num_stopped = blackboard.get('num_stopped_objects')
     */
    auto stopped_objects = tree.rootBlackboard()->get<ObjectList>("stopped_objects");
    auto num_stopped = tree.rootBlackboard()->get<int>("num_stopped_objects");

    std::cout << "\nFinal Results:\n";
    std::cout << "- Stopped objects detected: " << num_stopped << "\n";

    if (stopped_objects.size() > 0) {
        std::cout << "\nBroken-down vehicles to avoid:\n";
        for (const auto& obj : stopped_objects) {
            std::cout << "  → " << obj.id << " at " << obj.distance << "m "
                     << "(velocity: " << obj.velocity << " m/s, "
                     << "stopped: " << obj.stopped_duration << "s)\n";
        }
    }

    std::cout << "\n========================================\n";
    std::cout << "Expected Result:\n";
    std::cout << "- Should detect 2 broken-down vehicles (obj_2, obj_3)\n";
    std::cout << "- obj_1: filtered out (moving)\n";
    std::cout << "- obj_4: filtered out (too far)\n";
    std::cout << "- obj_5: filtered out (stopped < 2s)\n";
    std::cout << "========================================\n";

    return 0;
}

/**
 * BT Example: Stop for Broken-Down Vehicle
 *
 * This implements your BT diagram flow:
 * Detect Objects → Classify Objects → Decide (Avoid or Stop) → Execute Stop
 *
 * Uses code copied from Autoware static obstacle avoidance module
 */

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <chrono>

// ============================================================================
// COPIED FROM AUTOWARE: Data Structures
// ============================================================================

/**
 * Simplified PredictedObject (mimics ROS message)
 * In real implementation, use: autoware_perception_msgs::msg::PredictedObject
 */
struct Twist {
    struct Linear {
        double x, y, z;
    } linear;
};

struct Pose {
    struct Position {
        double x, y, z;
    } position;
};

struct PredictedObject {
    std::string object_id;
    Pose pose;
    Twist twist;
    double length{4.0};  // meters
};

using PredictedObjects = std::vector<PredictedObject>;

/**
 * ObjectData - COPIED FROM autoware data_structs.hpp (line 360)
 * Stores movement tracking info for each object
 */
struct ObjectData {
    PredictedObject object;

    // Position info (Frenet coordinates)
    double to_centerline{0.0};  // Lateral distance from lane center
    double longitudinal{0.0};   // Longitudinal distance from ego
    double length{0.0};

    // Movement tracking - KEY FOR BROKEN-DOWN VEHICLE DETECTION
    double move_time{0.0};      // How long object has been moving (seconds)
    double stop_time{0.0};      // How long object has been stopped (seconds)
    std::chrono::steady_clock::time_point last_stop;
    std::chrono::steady_clock::time_point last_move;

    Pose getPose() const { return object.pose; }

    double getVelocity() const {
        // COPIED FROM utils.cpp line 1780
        return std::hypot(object.twist.linear.x, object.twist.linear.y);
    }
};

using ObjectDataArray = std::vector<ObjectData>;

// ============================================================================
// COPIED FROM AUTOWARE: Utility Functions
// ============================================================================

/**
 * getObjectVelocity - COPIED FROM utils.cpp line 1780
 * Calculate 2D velocity magnitude from twist
 */
double getObjectVelocity(const PredictedObject & object)
{
    const auto & twist = object.twist;
    return std::hypot(twist.linear.x, twist.linear.y);
}

/**
 * isMovingObject - COPIED FROM utils.cpp line 244-250
 * Check if object has been moving long enough to be considered "moving"
 *
 * Returns:
 *  - true: Object is moving (ignore it, not broken-down)
 *  - false: Object stopped recently (might be broken-down vehicle)
 */
bool isMovingObject(const ObjectData & object, double moving_time_threshold)
{
    return object.move_time > moving_time_threshold;
}

/**
 * updateObjectMovementTracking - SIMPLIFIED FROM utils.cpp line 1772-1825
 *
 * Tracks object movement over time to detect broken-down vehicles
 *
 * Logic:
 * - If velocity < threshold for > 2.0s → stopped vehicle (broken-down!)
 * - If velocity > threshold → moving vehicle (not broken-down)
 *
 * This is the CORE function for detecting broken-down vehicles!
 */
void updateObjectMovementTracking(
    ObjectData & object_data,
    ObjectDataArray & stopped_objects_history,
    double moving_speed_threshold  // e.g., 1.0 m/s
)
{
    const auto velocity = object_data.getVelocity();
    const auto is_stopped = velocity < moving_speed_threshold;
    const auto & object_id = object_data.object.object_id;
    const auto now = std::chrono::steady_clock::now();

    // Find if we've seen this object before
    auto same_id_obj = std::find_if(
        stopped_objects_history.begin(),
        stopped_objects_history.end(),
        [&object_id](const auto & o) { return o.object.object_id == object_id; }
    );

    const bool is_new_object = (same_id_obj == stopped_objects_history.end());

    if (is_stopped) {
        // Object is currently stopped
        object_data.last_stop = now;
        object_data.move_time = 0.0;

        if (is_new_object) {
            // First time seeing this stopped object
            object_data.stop_time = 0.0;
            object_data.last_move = now;
            stopped_objects_history.push_back(object_data);
        } else {
            // Update stop duration
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - same_id_obj->last_move
            );
            same_id_obj->stop_time = duration.count() / 1000.0;
            same_id_obj->last_stop = now;
            same_id_obj->move_time = 0.0;

            object_data.stop_time = same_id_obj->stop_time;
        }
    } else {
        // Object is currently moving
        if (is_new_object) {
            object_data.move_time = std::numeric_limits<double>::infinity();
            object_data.stop_time = 0.0;
            object_data.last_move = now;
        } else {
            // Object was stopped, now moving
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - same_id_obj->last_stop
            );
            object_data.move_time = duration.count() / 1000.0;
            object_data.stop_time = 0.0;

            // If moved long enough, remove from stopped history
            const double moving_time_threshold = 2.0;
            if (object_data.move_time > moving_time_threshold) {
                stopped_objects_history.erase(same_id_obj);
            }
        }
    }
}

// ============================================================================
// BT NODES: Detection and Classification
// ============================================================================

/**
 * DetectObjects: Check if any objects are detected from perception
 */
class DetectObjects : public BT::ConditionNode
{
public:
    DetectObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<PredictedObjects>("perception_objects") };
    }

    BT::NodeStatus tick() override
    {
        auto objects_result = getInput<PredictedObjects>("perception_objects");

        if (!objects_result || objects_result.value().empty()) {
            std::cout << "[DetectObjects] ✗ No objects detected\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[DetectObjects] ✓ Detected " << objects_result.value().size()
                  << " objects\n";
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * ClassifyObjects: Classify objects as moving or stopped
 * Uses Autoware's updateObjectMovementTracking() function
 */
class ClassifyObjects : public BT::SyncActionNode
{
private:
    ObjectDataArray stopped_objects_history_;  // Track objects across frames
    const double MOVING_SPEED_THRESHOLD = 1.0;  // m/s (from Autoware config)

public:
    ClassifyObjects(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<PredictedObjects>("perception_objects"),
            BT::OutputPort<ObjectDataArray>("classified_objects")
        };
    }

    BT::NodeStatus tick() override
    {
        auto objects_result = getInput<PredictedObjects>("perception_objects");
        if (!objects_result) {
            return BT::NodeStatus::FAILURE;
        }

        PredictedObjects objects = objects_result.value();
        ObjectDataArray classified_objects;

        for (const auto & obj : objects) {
            ObjectData obj_data;
            obj_data.object = obj;

            // ← USE AUTOWARE FUNCTION HERE
            // This tracks object movement over time
            updateObjectMovementTracking(
                obj_data,
                stopped_objects_history_,
                MOVING_SPEED_THRESHOLD
            );

            classified_objects.push_back(obj_data);

            std::cout << "[ClassifyObjects] Object " << obj.object_id
                     << " - vel: " << obj_data.getVelocity() << " m/s"
                     << ", stop_time: " << obj_data.stop_time << "s"
                     << ", move_time: " << obj_data.move_time << "s\n";
        }

        setOutput("classified_objects", classified_objects);

        return BT::NodeStatus::SUCCESS;
    }
};

// ============================================================================
// BT NODES: Stop Decision
// ============================================================================

/**
 * DecideToStop: Decide if ego should stop for a broken-down vehicle
 *
 * Logic:
 * - If any object has been stopped for >= 2.0s → STOP
 * - Otherwise → CONTINUE
 */
class DecideToStop : public BT::ConditionNode
{
private:
    const double MOVING_TIME_THRESHOLD = 2.0;  // seconds
    const double STOP_TIME_THRESHOLD = 2.0;    // seconds

public:
    DecideToStop(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<ObjectDataArray>("classified_objects"),
            BT::OutputPort<bool>("should_stop"),
            BT::OutputPort<std::string>("stop_reason")
        };
    }

    BT::NodeStatus tick() override
    {
        auto objects_result = getInput<ObjectDataArray>("classified_objects");
        if (!objects_result) {
            return BT::NodeStatus::FAILURE;
        }

        ObjectDataArray objects = objects_result.value();

        for (const auto & obj : objects) {
            // Check if object is NOT moving (using Autoware function)
            if (!isMovingObject(obj, MOVING_TIME_THRESHOLD)) {
                // Object is stopped or just started moving

                if (obj.stop_time >= STOP_TIME_THRESHOLD) {
                    // Object has been stopped for >= 2.0s
                    // → This is a BROKEN-DOWN VEHICLE!

                    std::cout << "\n[DecideToStop] ⚠️  BROKEN-DOWN VEHICLE DETECTED!\n";
                    std::cout << "  → Object ID: " << obj.object.object_id << "\n";
                    std::cout << "  → Velocity: " << obj.getVelocity() << " m/s\n";
                    std::cout << "  → Stopped for: " << obj.stop_time << " seconds\n";
                    std::cout << "  → Decision: MUST STOP!\n\n";

                    setOutput("should_stop", true);
                    setOutput("stop_reason", "Broken-down vehicle ahead");

                    return BT::NodeStatus::SUCCESS;  // Yes, should stop!
                }
            }
        }

        std::cout << "[DecideToStop] ✓ No stopped vehicles → Continue\n";
        setOutput("should_stop", false);

        return BT::NodeStatus::FAILURE;  // No need to stop
    }
};

/**
 * SafetyCheck: Verify it's safe to stop
 * (Simplified - in real system, check rear collisions, deceleration limits)
 */
class SafetyCheck : public BT::ConditionNode
{
public:
    SafetyCheck(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // Simplified: Always safe in this example
        // Real implementation would check:
        // - Is there vehicle behind us?
        // - Can we decelerate safely without rear collision?
        // - Is deceleration within jerk limits?

        std::cout << "[SafetyCheck] ✓ Safe to stop (simplified check)\n";

        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * ExecuteStop: Command ego vehicle to stop
 */
class ExecuteStop : public BT::SyncActionNode
{
public:
    ExecuteStop(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("stop_reason"),
            BT::OutputPort<double>("target_velocity")
        };
    }

    BT::NodeStatus tick() override
    {
        auto reason_result = getInput<std::string>("stop_reason");
        std::string reason = reason_result.value_or("Unknown");

        std::cout << "\n[ExecuteStop] 🛑 STOPPING VEHICLE\n";
        std::cout << "  → Reason: " << reason << "\n";
        std::cout << "  → Setting target velocity = 0.0 m/s\n\n";

        double stop_velocity = 0.0;
        setOutput("target_velocity", stop_velocity);

        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * CheckStopComplete: Verify vehicle has stopped
 */
class CheckStopComplete : public BT::ConditionNode
{
public:
    CheckStopComplete(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("current_velocity") };
    }

    BT::NodeStatus tick() override
    {
        auto vel_result = getInput<double>("current_velocity");
        double velocity = vel_result.value_or(0.0);

        if (velocity < 0.1) {  // Nearly stopped
            std::cout << "[CheckStopComplete] ✓ Vehicle stopped successfully\n";
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[CheckStopComplete] ⏳ Still slowing down... (vel: "
                  << velocity << " m/s)\n";
        return BT::NodeStatus::RUNNING;
    }
};

// ============================================================================
// MAIN: Create and Run BT
// ============================================================================

int main()
{
    std::cout << "========================================\n";
    std::cout << "BT Example: Stop for Broken-Down Vehicle\n";
    std::cout << "Uses Autoware code for object tracking\n";
    std::cout << "========================================\n\n";

    // Create BT factory
    BT::BehaviorTreeFactory factory;

    // Register nodes
    factory.registerNodeType<DetectObjects>("DetectObjects");
    factory.registerNodeType<ClassifyObjects>("ClassifyObjects");
    factory.registerNodeType<DecideToStop>("DecideToStop");
    factory.registerNodeType<SafetyCheck>("SafetyCheck");
    factory.registerNodeType<ExecuteStop>("ExecuteStop");
    factory.registerNodeType<CheckStopComplete>("CheckStopComplete");

    // Define BT structure (based on your diagram)
    static const char* xml_text = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="StopForObstacle">
                <Sequence name="Root">
                    <!-- Step 1: Detect objects from perception -->
                    <DetectObjects
                        perception_objects="{perception_objects}" />

                    <!-- Step 2: Classify objects (moving vs stopped) -->
                    <ClassifyObjects
                        perception_objects="{perception_objects}"
                        classified_objects="{classified_objects}" />

                    <!-- Step 3: Decide if should stop -->
                    <Fallback name="DecideAvoidOrStop">
                        <!-- Option A: Avoid (Not implemented - future work) -->
                        <AlwaysFailure name="AvoidBehavior_NotImplemented" />

                        <!-- Option B: Stop Behavior -->
                        <Sequence name="StopBehavior">
                            <DecideToStop
                                classified_objects="{classified_objects}"
                                should_stop="{should_stop}"
                                stop_reason="{stop_reason}" />

                            <SafetyCheck />

                            <ExecuteStop
                                stop_reason="{stop_reason}"
                                target_velocity="{target_velocity}" />

                            <CheckStopComplete
                                current_velocity="{current_velocity}" />
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);

    // ========================================================================
    // Simulate perception data (broken-down vehicle scenario)
    // ========================================================================

    std::cout << "Test Scenario: Broken-down vehicle on road\n";
    std::cout << "========================================\n\n";

    // Simulate multiple frames (ticks)
    for (int frame = 1; frame <= 5; frame++) {
        std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
        std::cout << "Frame " << frame << " (simulated perception update)\n";
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

        // Create test objects
        PredictedObjects perception_objects;

        // Object 1: Moving car (not broken-down)
        PredictedObject moving_car;
        moving_car.object_id = "car_moving_001";
        moving_car.twist.linear.x = 10.0;  // 10 m/s forward
        moving_car.twist.linear.y = 0.0;
        moving_car.pose.position.x = 50.0;
        perception_objects.push_back(moving_car);

        // Object 2: Broken-down vehicle (stopped for multiple frames)
        PredictedObject broken_down_car;
        broken_down_car.object_id = "car_stopped_002";
        broken_down_car.twist.linear.x = 0.0;  // STOPPED!
        broken_down_car.twist.linear.y = 0.0;
        broken_down_car.pose.position.x = 30.0;
        perception_objects.push_back(broken_down_car);

        // Set blackboard data
        tree.rootBlackboard()->set("perception_objects", perception_objects);
        tree.rootBlackboard()->set("current_velocity", 5.0);  // Ego velocity

        // Tick the tree
        BT::NodeStatus status = tree.tickOnce();

        std::cout << "\n----------------------------------------\n";
        std::cout << "BT Status: ";
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
        std::cout << "----------------------------------------\n";

        // Check result
        auto should_stop = tree.rootBlackboard()->get<bool>("should_stop");
        auto target_velocity = tree.rootBlackboard()->get<double>("target_velocity");

        std::cout << "\nOutput:\n";
        std::cout << "  should_stop = " << (should_stop ? "true" : "false") << "\n";
        if (target_velocity != nullptr) {
            std::cout << "  target_velocity = " << target_velocity << " m/s\n";
        }

        // In frame 3+, the stopped object will have stop_time >= 2.0s
        // and BT should decide to STOP
        if (frame >= 3 && status == BT::NodeStatus::SUCCESS) {
            std::cout << "\n✓ SUCCESS: BT correctly detected broken-down vehicle and stopped!\n";
            break;
        }

        // Simulate time passing between frames
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "\n========================================\n";
    std::cout << "Test Complete!\n";
    std::cout << "========================================\n";

    return 0;
}

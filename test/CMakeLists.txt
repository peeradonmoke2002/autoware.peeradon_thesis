cmake_minimum_required(VERSION 3.16)
project(bt_obstacle_avoidance_test)

# Set C++ standard (BehaviorTree.CPP requires C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find BehaviorTree.CPP library
# Option 1: If installed via apt (Ubuntu):
#   sudo apt install libbehaviortree-cpp-dev
# Option 2: If built from source in Autoware workspace:
#   It will search in the workspace
find_package(behaviortree_cpp REQUIRED)

# Example 1: Simple BT (basic concepts)
add_executable(simple_bt_obstacle_avoidance
    simple_bt_obstacle_avoidance.cpp
)

target_link_libraries(simple_bt_obstacle_avoidance
    BT::behaviortree_cpp
)

# Example 2: BT with Stop Behavior (uses Autoware code)
add_executable(bt_stop_for_obstacle
    bt_stop_for_obstacle.cpp
)

target_link_libraries(bt_stop_for_obstacle
    BT::behaviortree_cpp
)

# Print information
message(STATUS "BehaviorTree.CPP found: ${behaviortree_cpp_FOUND}")
message(STATUS "Building BT examples:")
message(STATUS "  - simple_bt_obstacle_avoidance (basic example)")
message(STATUS "  - bt_stop_for_obstacle (with Autoware code)")

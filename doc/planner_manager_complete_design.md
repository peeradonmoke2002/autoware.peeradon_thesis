# Behavior Path Planner Manager: Complete Design (From Official Docs)

**Reference:** [Autoware Universe - Behavior Path Planner Manager Design](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_manager_design/)

**Date:** 2025-10-02

---

## Important Update to Our Understanding

After reading the official documentation, I need to correct our previous understanding. The manager is MORE sophisticated than we thought!

**Key New Insights:**
1. **Two Module Stacks:** Approved vs Candidate (not just "running" or "skipped")
2. **6-Step Processing Cycle:** More complex than simple "check and execute"
3. **RTC Approval System:** Modules need approval before path modification
4. **Simultaneous Execution Rules:** Complex logic for which modules can run together

---

## 1. The 6-Step Processing Cycle (Every 100ms)

### Official Flow

```
┌────────────────────────────────────────────────────────────────┐
│ STEP 1: Run Approved Modules                                  │
│                                                                │
│ - Execute modules that are already approved                    │
│ - Get their path modifications                                 │
│ - Check if any approved modules expired (completed/failed)     │
│ - Remove expired modules from approved stack                   │
│                                                                │
│ Output: approved_modules_output (modified path)                │
└──────────────────┬─────────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────────┐
│ STEP 2: Check Module Requests                                 │
│                                                                │
│ For each module in slot:                                       │
│   - Call module->isExecutionRequested()                        │
│   - If true → Add to request_modules list                      │
│   - If false → Skip this module                                │
│                                                                │
│ Output: request_modules (list of modules wanting to run)       │
└──────────────────┬─────────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────────┐
│ STEP 3: Evaluate Request Modules                              │
│                                                                │
│ If request_modules is empty:                                   │
│   → Return approved_modules_output (no changes)                │
│                                                                │
│ If request_modules is NOT empty:                               │
│   → Sort by priority                                           │
│   → Check which can execute simultaneously                     │
│                                                                │
│ Output: prioritized_request_modules                            │
└──────────────────┬─────────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────────┐
│ STEP 4: Select Candidate Modules                              │
│                                                                │
│ Apply simultaneous execution rules:                            │
│   - Check enable_simultaneous_execution_as_candidate_module    │
│   - Check if modules are compatible                            │
│   - Select highest priority executable modules                 │
│                                                                │
│ Output: candidate_modules (modules to execute this cycle)      │
└──────────────────┬─────────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────────┐
│ STEP 5: Execute Candidate Modules                             │
│                                                                │
│ For each candidate_module:                                     │
│   - Call module->isExecutionReady()                            │
│   - If ready → Call module->plan()                             │
│   - Generate candidate path                                    │
│   - Publish RTC status (request approval)                      │
│                                                                │
│ Output: candidate_paths (paths awaiting approval)              │
└──────────────────┬─────────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────────┐
│ STEP 6: Approve/Transition Modules                            │
│                                                                │
│ If RTC approved (or auto-approval enabled):                    │
│   - Move candidate_modules → approved_modules stack            │
│   - Clear candidate_module stack                               │
│   - Use new approved path                                      │
│                                                                │
│ If NOT approved:                                               │
│   - Keep candidate_modules in candidate stack                  │
│   - Continue using old approved_modules_output                 │
│                                                                │
│ Output: Final path to publish                                  │
└────────────────────────────────────────────────────────────────┘
```

---

## 2. Module State Machine

### Three States (Not Just Two!)

```
┌─────────────┐
│   IDLE      │  (Module loaded but not executing)
└──────┬──────┘
       │ isExecutionRequested() == true
       ↓
┌─────────────┐
│  CANDIDATE  │  (Requesting execution, generating candidate path)
└──────┬──────┘
       │ RTC Approval received
       ↓
┌─────────────┐
│  APPROVED   │  (Actively modifying path)
└──────┬──────┘
       │ Task completed or expired
       ↓
    (Back to IDLE)
```

### For Static Obstacle Avoidance Example:

```
Cycle 1 (t=0ms): No objects detected
  State: IDLE
  Action: None

Cycle 2 (t=100ms): Broken-down car detected!
  State: IDLE → CANDIDATE
  Action:
    - fillAvoidanceTargetObjects() → 1 object found
    - isExecutionRequested() → true
    - Added to request_modules
    - Sorted by priority
    - Selected as candidate
    - plan() called → generates avoidance path
    - RTC status published (requesting approval)

Cycle 3 (t=200ms): Approval received (auto or manual)
  State: CANDIDATE → APPROVED
  Action:
    - Candidate path approved
    - Module moved to approved_modules stack
    - Avoidance path published to vehicle control

Cycle 4-10 (t=300-1000ms): Still avoiding
  State: APPROVED
  Action:
    - Continue generating avoidance path
    - Vehicle executes lateral shift
    - Monitor completion

Cycle 11 (t=1100ms): Object passed, returned to center
  State: APPROVED → IDLE
  Action:
    - Task completed
    - Module removed from approved stack
    - Return to normal lane following
```

---

## 3. Approved vs Candidate Modules

### Approved Modules Stack

**What it is:**
- Modules currently executing and modifying the path
- Their output is being used by the vehicle
- Requires approval (RTC) to enter this stack

**Example:**
```cpp
approved_modules = [static_obstacle_avoidance]
                    ↑
                    Already approved, actively avoiding
```

**Characteristics:**
- Can continue running for multiple cycles
- Path modifications are actively applied
- Can be multiple modules simultaneously (if allowed)

### Candidate Modules Stack

**What it is:**
- Modules requesting to execute
- Generating "candidate paths" for preview
- Waiting for approval

**Example:**
```cpp
candidate_modules = [lane_change_left]
                     ↑
                     Requesting lane change, path generated but NOT executed yet
```

**Characteristics:**
- May or may not get approved
- Path shown as "candidate" (visualization in RViz)
- Requires RTC approval to move to approved stack

---

## 4. RTC (Request to Cooperate) System

### What is RTC?

**Purpose:** Safety mechanism for manual oversight of autonomous decisions

**How it works:**
1. Module generates candidate path
2. Publishes RTC status (request cooperation)
3. Operator sees candidate path in RViz
4. Operator approves or rejects
5. If approved → Module moves to approved stack
6. If rejected → Module stays in candidate (or cleared)

**Auto vs Manual Mode:**

```yaml
# Configuration: scene_module_manager.param.yaml
static_obstacle_avoidance:
  enable_rtc: false  # ← Auto-approval (no human needed)

# If enable_rtc: true
# - Manual approval required
# - Operator must click "approve" button
# - Module waits in candidate state until approval
```

**For your testing:**
- Keep `enable_rtc: false` → Auto-approval
- Module automatically moves from CANDIDATE → APPROVED
- No human intervention needed

---

## 5. Slot Management and Path Propagation

### What is a Slot?

**Definition:** A container that processes modules in priority order within the same stage

**Configuration (from `scene_module_manager.param.yaml`):**
```yaml
slots:
  - slot1
  - slot2
  - slot3
  - slot4

slot1:
  - "start_planner"

slot2:
  - "side_shift"
  - "avoidance_by_lane_change"
  - "static_obstacle_avoidance"
  - "lane_change_left"
  - "lane_change_right"
  - "bidirectional_traffic"

slot3:
  - "goal_planner"

slot4:
  - "dynamic_obstacle_avoidance"
```

### Path Propagation Flow

```
Input: Reference Path (from mission planner)
    ↓
┌─────────────────────────────────────────┐
│ Slot 1: start_planner                   │
│ - Modify path if starting from parking  │
│ - Output: path_slot1                    │
└──────────────┬──────────────────────────┘
               ↓ (path_slot1 becomes input to slot 2)
┌─────────────────────────────────────────┐
│ Slot 2: Multiple modules                │
│ Input: path_slot1                       │
│                                         │
│ - static_obstacle_avoidance checks path │
│   → If avoidance needed:                │
│     → Modify path (lateral shift)       │
│   → If not needed:                      │
│     → Pass through unchanged            │
│                                         │
│ - lane_change checks modified path      │
│   → Can run simultaneously if allowed   │
│                                         │
│ - Output: path_slot2 (combined mods)    │
└──────────────┬──────────────────────────┘
               ↓ (path_slot2 becomes input to slot 3)
┌─────────────────────────────────────────┐
│ Slot 3: goal_planner                    │
│ Input: path_slot2                       │
│ - Modify path if approaching goal       │
│ - Output: path_slot3                    │
└──────────────┬──────────────────────────┘
               ↓ (path_slot3 becomes input to slot 4)
┌─────────────────────────────────────────┐
│ Slot 4: dynamic_obstacle_avoidance      │
│ Input: path_slot3                       │
│ - Usually disabled                      │
│ - Output: path_slot4 (final)            │
└──────────────┬──────────────────────────┘
               ↓
        Final Path Published
```

**Key Points:**
- Slots process **sequentially** (Slot 1 → 2 → 3 → 4)
- Modules **within a slot** can run **simultaneously** (if allowed)
- Each slot's output becomes next slot's input
- Path is progressively modified through the pipeline

---

## 6. Simultaneous Execution Rules

### Configuration Parameters

```yaml
static_obstacle_avoidance:
  enable_simultaneous_execution_as_approved_module: true
  enable_simultaneous_execution_as_candidate_module: false

lane_change_left:
  enable_simultaneous_execution_as_approved_module: true
  enable_simultaneous_execution_as_candidate_module: true
```

### Execution Scenarios

#### Scenario 1: Both Want to Run

```
Input: Reference path
    ↓
Module: static_obstacle_avoidance
  - isExecutionRequested() → true (broken car detected)
  - State → CANDIDATE
    ↓
Module: lane_change_left
  - isExecutionRequested() → true (slower car ahead)
  - State → CANDIDATE
    ↓
Manager Checks:
  - Can both run as candidates?
    → static_obstacle_avoidance: enable_simultaneous_execution_as_candidate_module = false
    → lane_change_left: enable_simultaneous_execution_as_candidate_module = true
    → Result: NO! static_obstacle_avoidance blocks simultaneous candidate execution
    ↓
Priority Check:
  - Which has higher priority?
  - Assume static_obstacle_avoidance has higher priority
    ↓
Result:
  - static_obstacle_avoidance moves to CANDIDATE
  - lane_change_left is blocked (waits for next cycle)
```

#### Scenario 2: Both Already Approved

```
Approved Stack: [static_obstacle_avoidance, lane_change_left]
    ↓
Manager Checks:
  - Can both run as approved?
    → static_obstacle_avoidance: enable_simultaneous_execution_as_approved_module = true
    → lane_change_left: enable_simultaneous_execution_as_approved_module = true
    → Result: YES!
    ↓
Execution:
  - static_obstacle_avoidance generates lateral shift
  - lane_change_left generates lane change
  - Paths are merged (complex maneuver: "avoid obstacle while changing lane")
```

---

## 7. Module Priority

### Priority Determines Execution Order

**Configuration:**
```yaml
# Priority is determined by order in slot configuration
slot2:
  - "side_shift"                    # Priority 1 (highest)
  - "avoidance_by_lane_change"      # Priority 2
  - "static_obstacle_avoidance"     # Priority 3
  - "lane_change_left"              # Priority 4
  - "lane_change_right"             # Priority 5
  - "bidirectional_traffic"         # Priority 6 (lowest)
```

**When multiple modules request execution:**
```
Request modules: [lane_change_left, static_obstacle_avoidance, side_shift]
    ↓
Sort by priority:
    1. side_shift (priority 1)
    2. static_obstacle_avoidance (priority 3)
    3. lane_change_left (priority 4)
    ↓
Execute in order:
    1. Check side_shift first
    2. Then static_obstacle_avoidance
    3. Then lane_change_left
```

**Higher priority modules:**
- Checked first
- Execute first if multiple want to run
- Can block lower priority modules (depending on simultaneous execution settings)

---

## 8. Updated Execution Flow Diagram

### Complete Flow with All Details

```
Timer (100ms)
    ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 1: Run Approved Modules                              │
│                                                            │
│ approved_modules = [static_obstacle_avoidance]             │
│   └─ plan() → generates avoidance path                     │
│   └─ Check if completed → No, still avoiding               │
│                                                            │
│ Output: approved_path (with avoidance)                     │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 2: Check Module Requests (Slot 2)                    │
│                                                            │
│ For side_shift:                                            │
│   └─ isExecutionRequested() → false → SKIP                 │
│                                                            │
│ For avoidance_by_lane_change:                              │
│   └─ isExecutionRequested() → false → SKIP                 │
│                                                            │
│ For static_obstacle_avoidance:                             │
│   └─ Already in approved stack → Continue                  │
│                                                            │
│ For lane_change_left:                                      │
│   └─ isExecutionRequested() → true → ADD to request        │
│                                                            │
│ request_modules = [lane_change_left]                       │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 3: Evaluate Requests                                 │
│                                                            │
│ request_modules not empty → Sort by priority               │
│   [lane_change_left] (already only one)                    │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 4: Select Candidates                                 │
│                                                            │
│ Check simultaneous execution:                              │
│   - static_obstacle_avoidance (approved): allows simultaneous? YES│
│   - lane_change_left (candidate): allows simultaneous? YES │
│   - Compatible? → YES                                      │
│                                                            │
│ candidate_modules = [lane_change_left]                     │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 5: Execute Candidates                                │
│                                                            │
│ For lane_change_left:                                      │
│   └─ isExecutionReady() → Check safety → true             │
│   └─ plan() → Generate lane change path                    │
│   └─ Publish RTC status                                    │
│                                                            │
│ candidate_path = lane_change_path                          │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
┌────────────────────────────────────────────────────────────┐
│ STEP 6: Approve Candidates                                │
│                                                            │
│ RTC check:                                                 │
│   - enable_rtc = false → Auto-approve                      │
│   - Move lane_change_left to approved stack                │
│                                                            │
│ approved_modules = [static_obstacle_avoidance, lane_change_left]│
│                                                            │
│ Merge paths:                                               │
│   - Avoidance lateral shift + Lane change                  │
│   - Result: Complex maneuver path                          │
└──────────────────┬─────────────────────────────────────────┘
                   ↓
            Publish Final Path
```

---

## 9. Key Differences from Our Previous Understanding

| Previous Understanding | Actual Design |
|------------------------|---------------|
| Modules are just "running" or "skipped" | Three states: IDLE, CANDIDATE, APPROVED |
| Simple check and execute | 6-step processing cycle |
| Modules run independently | Approved + Candidate stacks with transitions |
| No approval mechanism | RTC approval system (optional) |
| Priority is simple ordering | Complex simultaneous execution rules |

---

## 10. Implications for Your BT Implementation

### What This Means:

**1. BT Node Structure Needs to Match Manager Flow:**

```
BehaviorTree: StaticObstacleAvoidanceModule
├─ Condition: isExecutionRequested()  ← STEP 2
│   ├─ HasAvoidableObjects?
│   └─ Return true/false
│
├─ Condition: isExecutionReady()  ← STEP 5
│   ├─ IsSafe?
│   ├─ IsComfortable?
│   └─ Return true/false
│
└─ Action: plan()  ← STEP 5
    ├─ GenerateAvoidancePath
    └─ Return path
```

**2. Module States Need Proper Management:**

Your BT doesn't control state transitions (IDLE → CANDIDATE → APPROVED).
The **PlannerManager** does that. Your BT just needs to:
- Return correct values from `isExecutionRequested()`
- Return correct values from `isExecutionReady()`
- Generate path in `plan()`

**3. RTC Integration:**

Keep `enable_rtc: false` for testing, so:
- CANDIDATE → APPROVED is automatic
- No manual approval needed
- Faster testing cycle

**4. Simultaneous Execution:**

Your BT module can run simultaneously with lane change if:
```yaml
static_obstacle_avoidance:
  enable_simultaneous_execution_as_approved_module: true
```

---

## 11. Summary

**The Planner Manager is like a sophisticated orchestra conductor:**

- **Slots** = Sections of the orchestra (strings, brass, etc.)
- **Modules** = Individual musicians
- **Approved Stack** = Currently playing musicians
- **Candidate Stack** = Musicians waiting to join
- **RTC** = Conductor approval gesture
- **Priority** = Which musicians play first
- **Simultaneous Execution** = Which musicians can play together

**Your BT implementation** = One musician (static_obstacle_avoidance)
**Manager** = Conductor that decides when you play

You don't need to change the conductor, just make your musician (BT) respond correctly to the conductor's signals (`isExecutionRequested()`, `isExecutionReady()`, `plan()`).

---

**End of Document**

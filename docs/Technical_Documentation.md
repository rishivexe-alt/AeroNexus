TECHNICAL PROJECT DOCUMENTATION

# Five-Drone Leader--Follower Formation

and Dynamic Multi-Drone Area Survey

ROS 2 Humble \| AeroStack2 \| Gazebo / Ignition \| DDS

System setup, software architecture, program development, operational
workflow, integration challenges and resolutions

Prepared for technical review and project handover

## Document Purpose and Scope

This document explains how the five-drone simulation project was built
from the initial software environment through the development of its
vehicle-control, formation, communication and task-management programs.
It is intended to give a mentor a technical account of the setup, design
decisions, program responsibilities, operating sequence, and integration
problems encountered.

The focus is on implementation and engineering workflow. Experimental
performance figures, measured errors, completion statistics and result
summaries are intentionally excluded.

The project programs were developed specifically for this system. The
document describes their design and operation; it does not claim that
the simulated system is a general-purpose flight-control framework or
that it has been validated on physical aircraft.

## Contents

-   1.  Project Overview and Objectives
-   2.  Software Configuration and Dependencies
-   3.  Workspace and Simulation Setup
-   4.  Five-Drone Startup
-   5.  Architecture and ROS 2 Communication
-   6.  Formation-Control Design
-   7.  Distributed Leader--Follower Formation
-   8.  Dynamic Survey Task Allocation
-   9.  Program Inventory
-   10. Operational Procedure
-   11. Engineering Problems and Resolutions
-   12. Diagnostics and Safe Shutdown
-   13. Design Boundaries
-   14. Handover Checklist
-   Appendices

## 1. Project Overview and Objectives

The project implements a five-vehicle multi-drone coordination
environment in simulation. The fleet consists of one leader, drone0, and
four followers, drone1 through drone4. The leader issues formation
commands, publishes its pose reference, assigns individual survey work
and interprets follower status messages.

The system was developed in layers so simulation, vehicle-level
behaviour, motion control, inter-process communication and mission
supervision could be developed and diagnosed independently.

### 1.1 Fleet roles

| Vehicle \| Role \| Responsibility \|

| --- \| --- \| --- \|

| drone0 \| Leader \| Formation reference, command interface, survey
  assignment and reassignment \|

| drone1 \| Follower 1 \| Formation follower; initial owner of survey
  area A \|

| drone2 \| Follower 2 \| Formation follower; initial owner of survey
  area B \|

| drone3 \| Follower 3 \| Formation follower; initial owner of survey
  area C \|

| drone4 \| Follower 4 \| Formation follower; helper for reassigned
  survey work \|

### 1.2 Objectives

-   Create a five-drone simulation with consistent namespaces and
    independent vehicle processes.

-   Implement takeoff, landing, line formation and V/arrow formation.

-   Develop a high-level control variant using the AeroStack2
    DroneInterface.

-   Develop direct feedback-based motion using pose feedback and Gazebo
    velocity topics.

-   Separate leader and follower execution into independent ROS 2
    processes communicating through DDS.

-   Assign survey waypoint sets to individual followers and receive
    progress notifications.

-   Implement a programmed delay scenario that transfers unfinished
    waypoints to an available follower.

## 2. Software Configuration and Dependencies

| Component \| Configuration / purpose \|

| --- \| --- \|

| Operating system \| Ubuntu 22.04 \|

| Middleware \| ROS 2 Humble \|

| Aerial robotics framework \| AeroStack2 \|

| Simulator \| Gazebo / Ignition Gazebo \|

| Python client \| rclpy \|

| Message packages \| geometry_msgs and std_msgs \|

| Inter-process communication \| ROS 2 topics over DDS / Fast DDS \|

| Process management \| Bash and tmux \|

| Programming languages \| Python 3 and Bash \|

### 2.1 Project directories

    ~/aerostack2_ws/                         # AeroStack2 workspace

    ~/project_gazebo/                         # project directory

    ~/project_gazebo/config/world_swarm_5drone.yaml

    # five-drone world configuration

    ~/project_gazebo/launch_as2_5drone.bash   # simulation launcher

The setup assumes ROS 2 Humble and the AeroStack2 workspace have already
been installed and built. The project launcher, world configuration and
Fast DDS profile must be present in the project directory.

### 2.2 Environment initialisation

Every terminal running a project node must source ROS 2 and AeroStack2.
The Fast DDS profile is set to the project's UDP-only profile.

    source /opt/ros/humble/setup.bash

    source ~/aerostack2_ws/install/setup.bash

    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"

    export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"

    export ROS2CLI_DISABLE_DAEMON=1

    cd ~/project_gazebo

The export statement and its variable assignment must remain on the same
shell line. The project scripts set both Fast DDS variable names for the
middleware configuration used during development.

### 2.3 Initial environment checks

    echo $ROS_DISTRO

    which ros2

    python3 --version

    ros2 topic list

    tmux list-sessions

## 3. Workspace and Simulation Setup

### 3.1 Prepare the workspace

1.  Open a terminal in Ubuntu 22.04.

2.  Source ROS 2 Humble and the installed AeroStack2 workspace.

3.  Change directory to \~/project_gazebo.

4.  Confirm that the five-drone world configuration and launcher exist.

5.  Confirm that the Fast DDS XML profile exists at the configured path.

    cd \~/project_gazebo

    ls

    ls config

    ls -l launch_as2_5drone.bash

    ls -l config/world_swarm_5drone.yaml

    ls -l config/fastdds_udp_only.xml

### 3.2 Launch the five-drone world

    cd ~/project_gazebo

    ./launch_as2_5drone.bash -m

The -m option selects multi-agent mode. The launcher starts the
configured simulation and associated vehicle sessions. The expected
namespaces are drone0, drone1, drone2, drone3 and drone4.

### 3.3 Inspect sessions, nodes and topics

    tmux list-sessions

    ros2 node list

    ros2 topic list

Separate tmux sessions make it possible to inspect vehicle startup
output and identify whether one vehicle process failed independently of
the others.

## 4. Five-Drone Startup

The simulated model, ROS 2 namespace and Python process are mapped
consistently. Drone0 is the leader and the remaining four namespaces are
followers.

### 4.1 Startup sequence

1.  Initialise the ROS 2 and AeroStack2 environment.

2.  Start the simulation using the multi-agent launcher.

3.  Inspect tmux sessions for the five vehicle sessions.

4.  Inspect ROS 2 nodes and topics.

5.  Confirm that each drone publishes ground-truth pose information.

6.  Start only one control implementation after the required interfaces
    are available.

### 4.2 Pose and platform-state inspection

    ros2 topic echo /drone0/ground_truth/pose --once

    ros2 topic echo /drone0/platform/info --once

    for d in 0 1 2 3 4; do

    echo "--- drone$d ---"

    timeout 8 ros2 topic echo /drone$d/ground_truth/pose --once

    done

Ground-truth pose feedback is used by the direct controller and the
DDS-based programs. Platform information is useful when diagnosing
vehicle lifecycle state and controller readiness.

## 5. Architecture and ROS 2 Communication

The architecture is divided into simulation, vehicle interface, motion
control, coordination and task-execution layers. ROS 2 topics form the
communication boundary between independent processes. The leader does
not call follower functions directly and does not share program memory
with follower processes.

### 5.1 Architecture layers

| Layer \| Responsibility \| Implementation \|

| --- \| --- \| --- \|

| Simulation \| Vehicle models, physics and simulator topics \| Gazebo /
  Ignition \|

| Vehicle abstraction \| Arming, offboard, takeoff, GoTo and landing \|
  AeroStack2 DroneInterface \|

| Motion control \| Pose feedback, target calculation and velocity
  generation \| direct_formation.py and DDS programs \|

| Coordination \| Leader commands and leader-pose distribution \|
  leader_follower_dds.py \|

| Task execution \| Individual survey tasks and waypoint tracking \|
  leader_tasks.py \|

| Task supervision \| Assignment, status interpretation and reassignment
  \| Leader logic in leader_tasks.py \|

### 5.2 Communication topics

| Topic \| Message type \| Purpose \|

| --- \| --- \| --- \|

| /formation/command \| std_msgs/String \| Formation command channel in
  DDS formation program \|

| /formation/leader_pose \| geometry_msgs/PoseStamped \| Leader's
  current pose reference \|

| /formation/task/droneN \| std_msgs/String \| Individual JSON task
  message for follower N \|

| /formation/status \| std_msgs/String \| Follower progress and
  task-status messages \|

| /droneN/ground_truth/pose \| geometry_msgs/PoseStamped \| Ground-truth
  position feedback \|

| /gz/droneN/cmd_vel \| geometry_msgs/Twist \| Direct Gazebo
  velocity-command interface \|

Task payloads are serialised as JSON and transported inside
std_msgs/String messages. A follower decodes the payload and selects the
corresponding task handler.

### 5.3 Leader and follower responsibilities

The leader publishes commands or individual task messages and receives
follower status updates. Each follower subscribes to its own task topic,
reads its own pose feedback, calculates its motion and publishes
commands for its own vehicle. In the DDS formation implementation,
followers also subscribe to the common leader-pose topic.

## 6. Formation-Control Design

### 6.1 Relative-offset model

Formation geometry is represented by fixed horizontal offsets relative
to the leader. Each follower calculates its target by adding its x and y
offsets to the leader reference position. A common target altitude is
used by the formation controller.

### 6.2 Line formation

| Vehicle \| x offset (m) \| y offset (m) \|

| --- \| --- \| --- \|

| drone0 (leader) \| 0.0 \| 0.0 \|

| drone1 \| -1.5 \| 0.0 \|

| drone2 \| -3.0 \| 0.0 \|

| drone3 \| -4.5 \| 0.0 \|

| drone4 \| -6.0 \| 0.0 \|

### 6.3 V / arrow formation

| Vehicle \| x offset (m) \| y offset (m) \|

| --- \| --- \| --- \|

| drone0 (leader) \| 0.0 \| 0.0 \|

| drone1 \| -1.5 \| 1.5 \|

| drone2 \| -1.5 \| -1.5 \|

| drone3 \| -3.0 \| 3.0 \|

| drone4 \| -3.0 \| -3.0 \|

### 6.4 Direct feedback controller

The direct-control implementation subscribes to each vehicle's
ground-truth pose and publishes geometry_msgs/Twist commands to the
corresponding Gazebo velocity topic. At each control iteration it
calculates the target-to-current position error.

    e_x = x_target - x_current

    e_y = y_target - y_current

    e_z = z_target - z_current

    v_x = clip(KP * e_x, VXY)

    v_y = clip(KP * e_y, VXY)

    v_z = clip(2 * KP * e_z, VUP if e_z > 0 else VDN)

| Parameter \| Value in direct_formation.py \|

| --- \| --- \|

| Target altitude \| 1.05 m \|

| Ground reference \| 0.06 m \|

| Proportional gain KP \| 0.8 \|

| Horizontal velocity limit \| 0.6 m/s \|

| Upward velocity limit \| 0.5 m/s \|

| Downward velocity limit \| 0.4 m/s \|

| Position tolerance \| 0.15 m \|

The clip operation bounds the velocity command. The vertical
proportional term is scaled by two, with separate upward and downward
velocity limits.

### 6.5 AeroStack2 high-level command variant

The AeroStack2 demonstration creates DroneInterface objects for all five
vehicles. It performs arming, offboard-mode activation and takeoff, then
uses GoTo behaviour calls to move the leader and followers to configured
coordinates. This variant is command-driven: the leader is held at a
nominal reference point and followers are not continuously tracking its
measured live pose.

### 6.6 Formation command lifecycle

1.  Initialise the five DroneInterface objects or the direct-control
    node.

2.  Issue TAKEOFF before formation commands.

3.  Select LINE or V_FORMATION / ARROW.

4.  Calculate each follower target from its configured offset.

5.  Continue the movement operation until the configured target
    condition is reached.

6.  Issue LAND before exiting the program.

## 7. Distributed Leader--Follower Formation

The DDS formation program separates the leader and followers into
independent processes. The leader publishes a formation command and its
current pose. Each follower receives the command, reads the leader pose,
selects its own offset and calculates its own target.

### 7.1 Process structure

| Process \| Node role \| Function \|

| --- \| --- \| --- \|

| Leader \| leader_drone0 \| Publishes formation command and leader pose
  \|

| Follower 1 \| follower_drone1 \| Subscribes to command, leader pose
  and own pose; publishes own velocity \|

| Follower 2 \| follower_drone2 \| Subscribes to command, leader pose
  and own pose; publishes own velocity \|

| Follower 3 \| follower_drone3 \| Subscribes to command, leader pose
  and own pose; publishes own velocity \|

| Follower 4 \| follower_drone4 \| Subscribes to command, leader pose
  and own pose; publishes own velocity \|

### 7.2 Leader-pose publication

The leader node reads its ground-truth position and periodically
publishes a PoseStamped message on /formation/leader_pose. The message
frame is labelled earth. Followers use its x, y and z values as the
reference for their formation targets.

### 7.3 Follower control loop

1.  Wait until the follower has received its own pose.

2.  Receive and store the leader pose.

3.  Receive the formation command and set the follower's operating mode.

4.  Select the follower-specific LINE or ARROW offset.

5.  Calculate target = leader pose + follower offset, using the
    configured altitude.

6.  Calculate position error and generate a clipped velocity command.

7.  Publish the command to the follower's Gazebo velocity topic.

### 7.4 Leader command interface

| Input \| Action \|

| --- \| --- \|

| T / TAKEOFF \| Take off to configured altitude \|

| 1 / LINE \| Select line formation \|

| 2 / ARROW \| Select V / arrow formation \|

| L / LAND \| Descend to ground reference \|

| g dx dy \| Move leader reference horizontally \|

| q \| Exit leader command loop \|

## 8. Dynamic Survey Task Allocation and Reallocation

The task-management program extends the system beyond group formation.
The leader assigns an individual survey task to selected followers. Each
follower executes its own waypoint list and reports progress. The leader
can transfer remaining waypoints to an available helper when the
programmed delay condition is reported.

### 8.1 Survey geometry and ownership

| Area \| Initial owner \| Waypoints (x, y), m \|

| --- \| --- \| --- \|

| A \| drone1 \| (-2.5,-2.0), (-2.5,2.0), (-1.0,2.0), (-1.0,-2.0) \|

| B \| drone2 \| (-0.5,-2.0), (-0.5,2.0), (0.5,2.0), (0.5,-2.0) \|

| C \| drone3 \| (1.0,-2.0), (1.0,2.0), (2.5,2.0), (2.5,-2.0) \|

| Helper \| drone4 \| Available for reassignment in the programmed delay
  workflow \|

The task program assigns different flight altitudes by follower index:
drone1 at 1.05 m, drone2 at 1.35 m, drone3 at 1.65 m and drone4 at 1.95
m.

### 8.2 Task message structure

The leader creates a publisher for each follower-specific task topic. A
task is converted to JSON and published to /formation/task/droneN. The
receiving follower parses the JSON and stores the task locally.

    {

    "type": "SURVEY",

    "area": "A",

    "wps": [[x1, y1, z], [x2, y2, z], [x3, y3, z], [x4, y4, z]],

    "todo": [0, 1, 2, 3]

    }

The todo array stores the waypoint indices that remain to be executed.
When a waypoint is reached within the configured tolerance, the follower
publishes WP_DONE and removes that index from its local todo list.

### 8.3 Supported task types

| Task \| Purpose \|

| --- \| --- \|

| TAKEOFF \| Move to takeoff altitude \|

| LAND \| Move to ground reference \|

| HOLD \| Hold at current or assigned target \|

| FORMATION \| Adopt LINE or ARROW formation \|

| GOTO \| Move to a specified coordinate \|

| SURVEY \| Execute a specified survey waypoint list \|

### 8.4 Status reporting

| Status event \| Meaning \|

| --- \| --- \|

| WP_DONE \| Waypoint reached; area and index reported \|

| DELAYED \| Programmed delay reported with remaining waypoint indices
  \|

| TASK_DONE \| Assigned task completed \|

| GOTO_DONE \| GOTO target reached \|

### 8.5 Delay-triggered reassignment workflow

1.  The leader assigns area A to drone1, area B to drone2 and area C to
    drone3.

2.  Drone4 is left available as a helper.

3.  In the programmed delay scenario, drone1 is configured to report a
    delay at a selected waypoint index.

4.  Drone1 reports its area and remaining waypoint indices on
    /formation/status.

5.  The leader receives DELAYED and checks its busy/available follower
    state.

6.  The leader selects an available helper and transfers the remaining
    waypoint list to it.

7.  The leader sends HOLD to the original follower.

8.  The helper executes the transferred waypoints and reports progress
    and task completion.

The delay is injected by the mission program through a delay_at field.
It is a controlled software scenario used to exercise reallocation
logic, not automatic detection of a real-world vehicle failure.

### 8.6 Leader task-state bookkeeping

The leader maintains busy-follower state, completed-waypoint state and
an area-owner mapping. These structures help determine which followers
are available and which waypoint indices have been reported complete.
The reassign routine selects a helper, sends it the remaining work and
sends HOLD to the source follower.

## 9. Program Inventory and Responsibilities

| File \| Purpose \| Execution role \|

| --- \| --- \| --- \|

| leader_follower_command_demo.py \| AeroStack2 high-level takeoff,
  formation and landing demonstration \| Single command process \|

| direct_formation.py \| Centralised pose-feedback controller using
  Gazebo velocity topics \| Single controller process \|

| leader_follower_dds.py \| DDS leader command and live leader-pose
  distribution \| Leader plus four followers \|

| leader_tasks.py \| Individual task assignment, status reporting and
  waypoint reassignment \| Leader plus four followers \|

| run_leader_follower.sh \| Environment setup and DDS follower process
  startup \| Bash launcher \|

| run_leader_tasks.sh \| Environment setup and task follower process
  startup \| Bash launcher \|

### 9.1 Launch-script operation

Each Bash launcher sources ROS 2 Humble and AeroStack2, exports the Fast
DDS profile variables, changes to the project directory and starts one
follower process for each index from 1 to 4. Output is redirected to
/tmp/followerN.log. A trap terminates background follower processes when
the leader process exits. A short startup delay is included before the
leader process starts.

    #!/bin/bash

    source /opt/ros/humble/setup.bash

    source ~/aerostack2_ws/install/setup.bash

    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"

    export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"

    cd ~/project_gazebo

    for i in 1 2 3 4; do

    python3 leader_follower_dds.py follower $i > /tmp/follower$i.log 2>&1 &

    done

    trap 'kill $(jobs -p) 2>/dev/null' EXIT

    sleep 3

    python3 leader_follower_dds.py leader

The task launcher uses the same process pattern but starts
leader_tasks.py instead of leader_follower_dds.py.

## 10. Operational Procedure

### 10.1 Start simulation

    cd ~/project_gazebo

    ./launch_as2_5drone.bash -m

    tmux list-sessions

### 10.2 AeroStack2 command demonstration

    source /opt/ros/humble/setup.bash

    source ~/aerostack2_ws/install/setup.bash

    cd ~/project_gazebo

    python3 leader_follower_command_demo.py

Commands: TAKEOFF, LINE, V_FORMATION, LAND and EXIT. TAKEOFF is required
before formation movement.

### 10.3 Direct formation controller

    cd ~/project_gazebo

    python3 direct_formation.py

Commands: TAKEOFF, LINE, V_FORMATION, LAND and EXIT. This variant uses
ground-truth pose feedback and direct Gazebo velocity commands.

### 10.4 DDS formation

    cd ~/project_gazebo

    chmod +x run_leader_follower.sh

    ./run_leader_follower.sh

Leader commands: T, 1, 2, L, g dx dy and q.

### 10.5 Task allocation

    cd ~/project_gazebo

    chmod +x run_leader_tasks.sh

    ./run_leader_tasks.sh

Leader commands include T, L, 1, 2, mission, mission delay, assign,
HOLD, LAND, LINE, ARROW, GOTO x y z, g dx dy, status and q. Use mission
delay to invoke the programmed delay/reassignment path.

### 10.6 Syntax validation

    python3 -m py_compile leader_follower_command_demo.py

    python3 -m py_compile direct_formation.py

    python3 -m py_compile leader_follower_dds.py

    python3 -m py_compile leader_tasks.py

Python bytecode compilation checks syntax only. It does not confirm that
ROS topics, Gazebo plugins, message transport or vehicle control are
functioning.

### 10.7 Inspect follower logs

    cat /tmp/follower1.log

    cat /tmp/follower2.log

    cat /tmp/follower3.log

    cat /tmp/follower4.log

## 11. Engineering Problems Encountered and Resolutions

Integration was diagnosed layer by layer rather than changing several
components at once. The main sequence was to confirm model creation,
inspect the ROS/Gazebo interface, check controller activation, verify
command delivery and finally inspect vehicle pose response.

### 11.1 Gazebo control-plugin identifier mismatch

Problem: An earlier model configuration used a plugin identifier that
did not match the installed Gazebo environment. The simulator did not
load the intended multicopter control system.

Investigation and resolution: The model SDF plugin declaration was
inspected against the simulator version. The Fortress-compatible
identifier ignition-gazebo-multicopter-control-system was used; the
earlier gz-sim-prefixed identifier was not valid in that environment.

### 11.2 Five-drone automatic spawning timed out

Problem: During early startup, automatic spawn requests for the five
vehicles timed out and the expected models were not visible.

Investigation: The spawn path was isolated from later control. The
Gazebo model list and create service were inspected. A direct ros_gz_sim
create operation was used to determine whether a model could be inserted
independently of the multi-agent launcher.

Resolution: Manual model creation was used as a diagnostic/recovery path
to isolate spawning, validate model files and confirm simulator model
creation. This was separate from the normal five-drone launcher.

### 11.3 SDF model-file validation

Problem: Invalid XML or an incorrect plugin declaration can prevent a
vehicle model from being inserted.

Resolution: Each generated SDF was parsed with Python's XML parser, and
the plugin declaration was searched directly.

    for i in 0 1 2 3 4; do

    python3 -c "

    import xml.dom.minidom as m

    m.parse('/tmp/quadrotor_base_$i.sdf')

    print('$i XML OK')

    "

    done

    grep -n -E "multicopter-control|MulticopterVelocityControl" /tmp/quadrotor_base_0.sdf

### 11.4 Vehicle exists but does not respond to velocity commands

Problem: A vehicle can be present in Gazebo while not responding to
velocity commands.

Investigation: The command path was checked in stages: publisher,
bridge/transport topic, velocity-controller activation and observed
pose.

Resolution: The controller-enable topic was checked and a minimal
vertical velocity command was used diagnostically, followed by a
zero-velocity command.

    ign topic -t /model/drone0/velocity_controller/enable \

    -m ignition.msgs.Boolean -p 'data: true'

    ign topic -t /model/drone0/cmd_vel \

    -m ignition.msgs.Twist -p 'linear: {z: 0.5}'

    ign topic -t /model/drone0/cmd_vel \

    -m ignition.msgs.Twist -p 'linear: {z: 0.0}'

These are simulator diagnostics, not a replacement for the mission
programs or a physical flight procedure.

### 11.5 ROS 2 / Gazebo topic and bridge diagnosis

Problem: A ROS publisher may exist while the command fails to reach the
Gazebo control interface because the expected topic or bridge is missing
or incorrectly mapped.

Resolution: Inspect ROS and Gazebo topic lists separately, inspect topic
information and echo one pose message. Confirm the command topic exists
under the expected namespace.

    ros2 topic list

    ign topic -l

    ign topic -i -t /model/drone0/cmd_vel

    ign topic -e -n 1 -t /model/drone0/pose

    ros2 topic echo /drone0/ground_truth/pose --once

### 11.6 Simulation-time and real-time-factor differences

Problem: Wall-clock duration does not necessarily equal simulation-time
duration when Gazebo runs below real time. This affects how movement
duration and timeout behaviour are interpreted.

Resolution: Gazebo's real-time factor was considered during diagnosis
instead of assuming that wall-clock seconds always correspond to
simulation seconds.

### 11.7 Independent process startup and observability

Problem: In a multi-process system, it can be difficult to identify
whether a fault originates in the leader, one follower or the transport
layer.

Resolution: Each follower was run as an independent process, with output
redirected to its own /tmp/followerN.log file. tmux sessions, ROS node
lists, topic lists and per-drone pose checks were used to isolate
process-level problems.

### 11.8 Task reassignment and task-state coordination

Design challenge: Reassignment requires the leader to know which
followers are busy, which waypoints are complete and which indices
remain.

Implementation: The leader maintains busy and completed-waypoint state.
Followers report WP_DONE, DELAYED and TASK_DONE. On DELAYED, the leader
selects an available helper, transfers the remaining waypoint indices
and sends HOLD to the source follower.

Scope: The delay is injected by mission code at a configured waypoint
index. It demonstrates the reallocation pathway; it is not a
sensor-based failure detector.

### 11.9 Command ordering and mission lifecycle

Design challenge: Formation and survey commands depend on the vehicles
being in the appropriate flight state. Running multiple control variants
simultaneously can also create competing commands.

Resolution: Start the simulation first and run only one control variant
at a time. The command demonstrations require takeoff before formation
movement, and the shutdown procedure requires landing before stopping
the simulation.

## 12. Diagnostics, Maintenance and Safe Shutdown

### 12.1 Diagnostic commands

    ign model --list

    ign topic -l

    ros2 node list

    ros2 topic list

    ros2 topic echo /drone0/ground_truth/pose --once

    ros2 topic echo /drone0/platform/info --once

    tmux list-sessions

    cat /tmp/follower1.log

    cat /tmp/follower2.log

    cat /tmp/follower3.log

    cat /tmp/follower4.log

### 12.2 Fleet-wide pose inspection

    for d in 0 1 2 3 4; do

    echo "--- drone$d ---"

    timeout 8 ros2 topic echo /drone$d/ground_truth/pose --once

    done

### 12.3 Safe shutdown

1.  Issue LAND through the active control program.

2.  Check the simulated vehicle states and confirm that the vehicles
    have reached the ground reference.

3.  Exit the leader/control program after the landing command has been
    handled.

4.  Stop the simulation using the project stop script.

    cd \~/project_gazebo

    ./stop.bash

A complete tmux server reset was used only when a full session reset was
required. It should not replace sending a landing command to active
simulated vehicles.

## 13. Design Boundaries and Assumptions

-   The project described here is a simulation implementation using ROS
    2 Humble, AeroStack2 and Gazebo / Ignition.

-   Formation geometries are predefined offsets in the configured earth
    frame.

-   The direct-control variants use simulator ground-truth pose
    feedback; this is not equivalent to onboard real-world state
    estimation.

-   The delay condition is programmed into the mission workflow and is
    not a general fault-detection mechanism.

-   Helper selection is based on the leader's software-maintained
    busy/available state.

-   Task messages use JSON encoded inside std_msgs/String messages.

-   The AeroStack2 command demonstration and DDS live leader-pose
    implementation have different control behaviours and are separate
    program variants.

-   Python syntax validation alone does not replace runtime integration
    checks.

## 14. Handover Checklist

-   -   [ ] ROS 2 Humble is installed and sourceable.
-   -   [ ] AeroStack2 workspace is built and sourceable.
-   -   [ ] Project root and five-drone world configuration are present.
-   -   [ ] Fast DDS profile path is valid and environment variables are
        exported correctly.
-   -   [ ] Five-drone launcher creates the expected vehicle namespaces
        and sessions.
-   -   [ ] All five ground-truth pose topics are available.
-   -   [ ] Only one control variant is active at a time.
-   -   [ ] All four Python files pass py_compile syntax validation.
-   -   [ ] Both Bash launch scripts are executable.
-   -   [ ] Follower log files can be inspected independently.
-   -   [ ] Task assignment, status reporting and programmed
        reassignment logic are understood.
-   -   [ ] Landing is commanded before stopping the simulation.

## Appendix A -- Key Command Reference

| Purpose \| Command \|

| --- \| --- \|

| Load ROS 2 \| source /opt/ros/humble/setup.bash \|

| Load AeroStack2 \| source \~/aerostack2_ws/install/setup.bash \|

| Enter project \| cd \~/project_gazebo \|

| Launch five drones \| ./launch_as2_5drone.bash -m \|

| List tmux sessions \| tmux list-sessions \|

| AeroStack2 command demo \| python3 leader_follower_command_demo.py \|

| Direct formation \| python3 direct_formation.py \|

| DDS formation \| ./run_leader_follower.sh \|

| Task allocation \| ./run_leader_tasks.sh \|

| Stop simulation \| ./stop.bash \|

## Appendix B -- Source Files Covered

The following six programs form the source set described in this
document:

-   leader_follower_command_demo.py

-   direct_formation.py

-   leader_follower_dds.py

-   run_leader_follower.sh

-   leader_tasks.py

-   run_leader_tasks.sh

The source files are maintained separately. This document explains their
architecture and operation rather than duplicating the full source
listings.

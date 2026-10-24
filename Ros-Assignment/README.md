# Ros Assignment – Lab 1: ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems – Faculty of Engineering, Alexandria University

## Team

| Name | ID |
|------|----|
| Mohamed Sherif Mohamed | 9550 |
| David Victor Kamal | 9298 |
| Karim Ahmed Elgohary | 9313 |

Developed for ROS 2 Humble (Ubuntu 22.04).

## Packages

| Package | Type | Contents |
|---------|------|----------|
| `turtle_interfaces` | ament_cmake | `msg/TargetPose` (x, y, yaw), `srv/SetSpeedLimits`, `srv/ResetTurtle`, `action/GoToPose`, `action/DrawSquare` |
| `turtle_motion` | ament_python | `go_to_pose_server` (action `/go_to_pose` + speed parameters + service `/set_speed_limits`), `reset_service` (service `/reset_turtle`), `square_action_server` (action `/draw_square`), launch files and the speed YAML |

### How the nodes work together

```
 turtlesim_node  <-- /turtle1/cmd_vel -- go_to_pose_server (params: max_linear_speed, max_angular_speed)
                 --- /turtle1/pose ---->      ^   (service /set_speed_limits)
                                              |  /go_to_pose action
 square_action_server  (/draw_square) --------+
        |  /reset_turtle service
        v
 reset_service --> turtlesim: /turtle1/set_pen, /turtle1/teleport_absolute, /clear
```

* **goToPose**: drives toward (x, y) with a proportional controller (turns in place first if the heading error is large), then rotates to the final yaw. Tolerances: 0.05 m and 0.02 rad. Publishes feedback (distance, current pose), rejects goals outside the 0–11 window, and re-reads the speed parameters on **every** control cycle so changes apply live.
* **Reset service**: lifts the pen, teleports to (5.544445, 5.544445, 0), puts the pen down, then calls `/clear`. Only turtlesim's own services are used.
* **Square**: calls `/reset_turtle`, then sends exactly 4 sequential `GoToPose` goals (one per corner) and waits for each. Feedback = current side. `side_length` of `0` means the default (2.0); max is 5.3 so the square stays inside the window.

## Build (from a fresh workspace)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repository's URL> .      # green "Code" button on GitHub -> copy the URL
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y     # optional
colcon build
source install/setup.bash
```

Every new terminal must run `source ~/ros2_ws/install/setup.bash`.

## Run everything with one command

```bash
ros2 launch turtle_motion system.launch.py
```

Starts turtlesim, `go_to_pose_server`, `reset_service` and `square_action_server`, with speed limits from `turtle_motion/config/speed_params.yaml`.

Start the system **and** draw a square automatically:

```bash
ros2 launch turtle_motion square.launch.py side_length:=3.0
```

Use another parameter file:

```bash
ros2 launch turtle_motion system.launch.py params_file:=/path/to/my_params.yaml
```

## Commands to try every requirement

(with `system.launch.py` running; use new terminals)

| Req | Command | Expected |
|-----|---------|----------|
| R1 | `ros2 node list` | `/turtlesim`, `/go_to_pose_server`, `/reset_service`, `/square_action_server` |
| R2 | `ros2 interface show turtle_interfaces/msg/TargetPose` | `float64 x`, `float64 y`, `float64 yaw` |
| R3 | `ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target_pose: {x: 8.0, y: 5.5, yaw: 0.0}}" --feedback` | feedback while moving, then `success: true` |
| R3 | `ros2 topic echo /turtle1/pose --once` | pose ≈ goal |
| R4 | `ros2 service call /reset_turtle turtle_interfaces/srv/ResetTurtle "{}"` | screen cleared, turtle at (5.54, 5.54, 0), no line drawn |
| R5 | `ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback` | feedback `current_side: 1..4`, square drawn; run twice |
| R6 | `ros2 launch turtle_motion system.launch.py` | all nodes start with nothing started by hand |
| R7 | `ros2 param list /go_to_pose_server` | `max_linear_speed`, `max_angular_speed` |
| R7 | `ros2 param get /go_to_pose_server max_linear_speed` | value from the YAML (2.0) |
| R7 | `ros2 param set /go_to_pose_server max_linear_speed 0.5` | `Set parameter successful`, turtle becomes slower |
| R7 | `ros2 param set /go_to_pose_server max_linear_speed -1.0` | rejected |
| R7 | `ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.5}"` | `success: true` |
| R7 | `ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: -2.0, max_angular_speed: 1.0}"` | `success: false` (parameters unchanged) |
| R8 | `ros2 pkg list \| grep turtle_` | `turtle_interfaces`, `turtle_motion` |

Other useful checks: `ros2 action list`, `ros2 service list | grep -E "reset|speed"`, `ros2 interface list | grep turtle_interfaces`.

## Bonus – design patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **Facade** | `reset_service.py`, `square_action_server.py` | `/reset_turtle` hides three turtlesim calls (`set_pen`, `teleport_absolute`, `clear`) behind one simple service. `/draw_square` hides the reset call and the four `goToPose` goals behind one action. Callers never deal with the details. |
| **Observer** | `go_to_pose_server.py` | The server *observes* the turtle through the `/turtle1/pose` subscription (ROS publish/subscribe), and clients observe the server through action feedback. The `on_set_parameters` callback also observes parameter changes and validates them, so speed limits can be changed live. |
| **State machine (lightweight)** | `go_to_pose_server.py` | Each goal runs through two states, DRIVE (go to x, y) and ROTATE (turn to the final yaw), then finishes as succeeded, cancelled or aborted. Keeping the states separate makes the controller easy to follow and tune. |
| **Single responsibility / Strategy-style separation** | whole package | The movement controller (`goToPose`) is separate from the orchestration (`draw_square`) and from turtlesim housekeeping (`reset_turtle`). Square reuses goToPose through its action interface instead of duplicating motion code. |

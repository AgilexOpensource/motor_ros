# motor_ros

ROS 2 bridge for an `agx_motor_sdk` **ArmJointGroup** YAML module: position/MIT
commands, joint feedback, motor diagnostics and enable services. All interfaces
use standard ROS messages.

> **Safety:** Default startup configures and enables motors and accepts commands.
> Check wiring, node IDs, calibration, joint limits and mechanical clearance
> before starting. Use settings verified for your hardware and provide an
> independent emergency stop.

## Installation

Requires ROS 2, a matching Python interpreter, and the SDK's Linux SocketCAN
backend. Load ROS 2 and activate your Python environment first. Use the same
Python environment to install the SDK, build and run the package.

### 1. Install the SDK dependency

```bash
python -m pip install \
  "git+https://github.com/kehuanjack/agx_motor_sdk.git"
```

### 2. Create a workspace and clone the source

```bash
mkdir -p ~/motor_ws/src
cd ~/motor_ws/src
git clone https://github.com/AgilexOpensource/motor_ros.git
```

### 3. Build and source the workspace

```bash
cd ~/motor_ws
colcon build --packages-select motor_ros --symlink-install
source install/setup.bash
```

## Quick Start

The examples use [default.yaml](config/default.yaml), module `motor`, joint
`joint`, namespace `/motor` and CAN interface `can0`. Adjust them for your hardware.

### 1. Activate CAN

Activate the intended adapter at 1 Mbps. Replace `can0` with your interface;
do not reconfigure an adapter in use.

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 2. Start the bridge

Terminal 1: choose either launch or direct node startup.

**Launch:**

```bash
ros2 launch motor_ros motor.launch.py namespace:=motor can_channel:=can0
```

**Node:**

```bash
ros2 run motor_ros motor_node --ros-args -r __ns:=/motor \
  -p yaml_path:=default.yaml -p module_key:=motor -p can_channel:=can0
```

### 3. Check feedback

Terminal 2: view joint positions, velocities and torques:

```bash
ros2 topic echo /motor/feedback/joint_states
```

Or view motor enable state, faults, voltage and temperatures:

```bash
ros2 topic echo /motor/feedback/motor_status
```

### Optional: connection checks

For connection checks without automatic configuration, enable or external
commands, use this launch instead. Restart with normal settings before control.

```bash
ros2 launch motor_ros motor.launch.py namespace:=motor can_channel:=can0 \
  auto_configure:=false auto_enable:=false control_enabled:=false
```

## Control and Feedback

Run commands in another terminal with ROS 2 and the workspace environment loaded.
**Commands can move the motor.** Use your configured joint names and safe targets;
the examples below command `joint` to 0 rad.

### Position control

```bash
ros2 topic pub --once /motor/control/joint_states sensor_msgs/msg/JointState \
  "{name: ['joint'], position: [0.0]}"
```

### MIT control

Switch mode, then send joint position (rad), velocity (rad/s) and torque (N*m).
MIT gains and torque calibration are defined in the motor YAML.

```bash
ros2 param set /motor/motor_node control_mode mit
ros2 topic pub --once /motor/control/joint_states sensor_msgs/msg/JointState \
  "{name: ['joint'], position: [0.0], velocity: [0.0], effort: [0.0]}"
```

### Motor enable and command gate

`enable` controls motor enable state. `control_enable` controls whether the bridge
accepts joint commands. To enable motors and open the command gate:

```bash
ros2 service call /motor/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /motor/control_enable std_srvs/srv/SetBool '{data: true}'
```

To close the gate and disable motors:

```bash
ros2 service call /motor/control_enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /motor/enable std_srvs/srv/SetBool '{data: false}'
```

### Interfaces

Names below are relative to the namespace. Topics use reliable, volatile QoS
with depth 10; services use standard ROS service QoS.

| Direction | Name | Type |
|---|---|---|
| Subscribe | `control/joint_states` | `sensor_msgs/JointState` |
| Publish | `feedback/joint_states` | `sensor_msgs/JointState` |
| Publish | `feedback/motor_status` | `diagnostic_msgs/DiagnosticArray` |
| Service | `enable` | `std_srvs/SetBool` |
| Service | `control_enable` | `std_srvs/SetBool` |

**Command contract:**

- `name` and `position`: nonempty, equal length, no duplicate names. Subsets are
  allowed; unknown joints are skipped. Position is joint-space rad.
- `pos` ignores velocity/effort. `mit` uses velocity (rad/s) and effort (N*m);
  each array must be empty (zeros) or match `name` length.
- Used values must be finite. Invalid arrays or NaN/Inf reject the whole packet
  before sending. Validated writes are per joint, not an atomic CAN transaction;
  a failed write is logged and other joints continue.
- Command headers are ignored; there is no age check, acknowledgment or replay.

**Joint feedback:** Positions, velocities and calibrated torques are published
from cached SDK snapshots in rad, rad/s and N*m. Missing joints are omitted;
no message is sent if all are missing. Snapshots may repeat and are not expired.
The default 100 Hz is the publication rate. Both feedback topic headers use ROS
publication time.

## Configuration Reference

### Motor YAML

Motor YAML defines the CAN interface, joints, node IDs (1..15), polarity, limits,
calibration and MIT gains. See the [bundled examples](config/) and the
[SDK YAML reference](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md).

Relative `yaml_path` values resolve under installed `share/motor_ros/config`.
Use absolute paths for external files; `package://` is not supported. An empty
node path checks `MOTOR_ROS_CONFIG`, then `default.yaml`. Supply `module_key` to
select the top-level YAML module; otherwise its name is inferred from the file
stem. For `default.yaml`, explicitly use `module_key:=motor`.

Optional per-joint `torque_limit` (N*m, finite and nonnegative) is applied during
automatic configuration: `current_a = torque_limit / (torque_k * torque_b)`.
The calibration product must be finite and positive. This sets a calibrated
current limit, not a measured-torque safety guarantee.

### ROS parameters

[motor_ros.yaml](config/motor_ros.yaml) supplies the default launch parameters.
Motor YAML describes hardware; ROS parameter YAML configures the bridge.

| Parameter | Node default | Meaning |
|---|---|---|
| `yaml_path` | empty | Motor YAML; default launch supplies `default.yaml` |
| `module_key` | empty | Module key; default launch supplies `motor` |
| `can_channel` | empty | Override YAML CAN interface; default launch supplies `can0` |
| `auto_configure` | `true` | Configure group and optional current limits |
| `auto_enable` | `true` | Enable motors at startup |
| `control_enabled` | `true` | Initial software command gate |
| `control_mode` | `pos` | `pos` or `mit`; dynamically changeable |
| `feedback_rate` | `100.0` | Joint publication frequency, Hz |
| `status_rate` | `2.0` | Diagnostic publication frequency, Hz |
| `status_timeout` | `2.0` | Low-speed freshness bound, seconds |

Rates and timeout must be finite and positive. All parameters except
`control_mode` are startup-only/read-only. Change the gate via `control_enable`;
its parameter keeps the startup value, while diagnostics show the current gate.

Launch accepts these overrides plus `namespace` and `params_file`. Empty overrides
preserve the parameter file. Selecting `yaml_path` infers `module_key` from its
stem unless explicitly supplied. For custom motor configuration:

```bash
ros2 launch motor_ros motor.launch.py namespace:=motor can_channel:=can0 \
  yaml_path:=/absolute/path/to/motor.yaml module_key:=motor
```

To select a ROS parameter file, pass `params_file:=/absolute/path/to/params.yaml`.
Inspect launch arguments with `ros2 launch motor_ros motor.launch.py --show-args`.

### Motor diagnostics

Diagnostics include every configured joint, with `name` = joint name and
`hardware_id` = `<can_channel>:<node_id>`. KeyValue fields are strings:

| Fields | Meaning |
|---|---|
| `enabled` | Hardware enable bit: `true`, `false`, `unknown` |
| `undervoltage`, `motor_overtemp`, `driver_overcurrent`, `driver_overtemp` | Fault bits |
| `collision_tripped`, `driver_error`, `stall_tripped` | Protection/error bits; stall bit reserved in the arm protocol |
| `bus_voltage_v`, `bus_current` | Bus voltage (V), bus current (A) |
| `driver_temp_deg`, `motor_temp_deg` | Temperatures (degrees C) |
| `status_raw` | Raw firmware status byte, decimal |
| `control_mode` | Current software control mode |

Fresh, fault-free status is `OK` (0), even if disabled; faults produce `ERROR` (2).
Missing, nonpositive, future or expired timestamps produce `STALE` (3), with
hardware values `unknown`; no `WARN` is emitted. Freshness is
`0 <= age <= status_timeout` using host wall-clock time, not ROS simulation time.
Timeout is informational, not a watchdog.

### Operating boundaries

- Closing the command gate does not disable motors or cancel targets already sent.
- Stale diagnostics do not stop commands or expire cached joint feedback.
- Node exit closes the SDK connection without explicitly disabling motors.
- The enable service reports the SDK result; check diagnostics for hardware state.
- Provide command watchdogs, trajectory execution, arbitration and recovery upstream.

## License

[Apache-2.0](LICENSE). SDK prebuilt protocol libraries retain their proprietary,
AgileX-only restrictions.

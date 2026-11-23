# ROS2_LIBXR navigation bridge

This package adapts the navigation workspace to the MC02 firmware SharedTopic
protocol. It is built around the upstream `ROS2_LIBXR` repository and retains
its LibXR transport implementation.

## Topics

The bridge subscribes to `/cmd_vel_mpc` and `/sentry/behaivor_send` and
publishes `nav_data` and `behavior_data` at 100 Hz over navigation USB CDC.
`nav_data` uses a 13-byte packed velocity protocol: three float32 values for
chassis-frame `vx`, `vy` (m/s) and `wz` (rad/s), followed by a validity flag. The bridge rotates
the MPC's `camera_init` velocity using LIO yaw plus `chassis_yaw_offset` and
projects heading over odometry age plus `rotate_latency`.

Commands are unstamped; both ROS and steady-clock time since receipt must be
within 150 ms. Odometry must match `odom_frame` and be fresh within 300 ms by
both clocks. Missing, stale, non-finite or unrepresentable inputs produce a zero
payload with the speed-control flag cleared; the matching firmware revokes AI
chassis control. A fresh intentional zero velocity keeps the flag set.

MC02 firmware must include the matching HostDataLink, DualBoard and Chassis
module revisions and both boards must be updated together. Verify the yaw offset and chassis axis convention on
hardware before driving.

## Tunnel preparation and escape

`MincoMpcController` applies the ROS Behavior tunnel policy before publishing
`/cmd_vel_mpc`. When `tunnel_escape_active` is true, it replaces the navigation
XY command with the world-frame escape XY command. When
`tunnel_prepare_active && !tunnel_ready`, it multiplies both selected XY
components by 0.15. This keeps decision behavior in the controller output and
leaves this bridge responsible for freshness checks, world-to-chassis rotation,
and protocol transmission.

## Navigation rotor speed

With fresh `use_gyro_mode` enabled and valid navigation and odometry, the bridge
converts the signed `Behavior.gyro_vel` from rpm to rad/s and sends it in
`nav_data.wz`. The chassis applies its angular-speed limit and rotor scaling.
An invalid gyro speed or a Behavior timeout of 300 ms clears gyro mode and
restores the MPC yaw command. Update the bridge and chassis firmware together
for this behavior.

The bridge receives `team_info`, `game_info`, `online_info`, `offline_info`,
and `radar_info`, and publishes the corresponding ROS status topics. Behavior angles are converted to radians.

## Run

`start.bash` starts the bridge by default. Set `LIBXR_BRIDGE_ENABLE=0` to run
without the physical USB link. For standalone launch:

```bash
ros2 launch ros2_libxr ros2_libxr_launch.py
```

Parameters include `cmd_topic`, `command_timeout`, `odom_topic`, `odom_frame`,
`odom_timeout`, `rotate_latency`, `chassis_yaw_offset`, `behavior_topic`, `baudrate` (921600),
`usb_vid` (`16d0`), `usb_pid` (`1492`), and `control_interface`
(`XRUSB Navigation Control`).

## Protocol caveats

The firmware's enemy position fields are unsigned 16-bit values with no unit
declared in the current wire definition. They are copied numerically into ROS
position fields; consumers must not treat them as meters until firmware defines
the scale. The current firmware also does not transmit several fields available
in the ROS Behavior and offline status messages.

## 2027 referee telemetry migration

Update the chassis, gimbal and this ROS workspace together. The referee CAN
layout is version 2. `game_info` is now exactly 30 bytes: its previous field
order is retained, `events` is a little-endian 64-bit value carrying both
0x0101 words, and the final fields are the two little-endian uint16 control
degrees (ally, enemy). The former 22-byte payload is rejected by the bridge;
rolling back requires all three peers to return to matching versions.
`ros_interfaces/msg/GameInfo.idl` and `.msg` carry the same widened event word
and appended control degrees. Rebuild the message package and all consumers.

`online_info` remains 37 bytes. The former energy percentage is unavailable in
the 2027 referee protocol and is sent as 255. ROS forwards that sentinel; the
behavior tree stores `ENERGY_UNKNOWN` instead of treating it as 255% or 0%.
It does not force MOVE because energy is unknown, and energy-dependent
reinforcement conditions remain unavailable until known feedback exists.

The bridge decodes fixed-layout telemetry only when the incoming payload has
the exact required length. The host tests cover literal 30-byte payloads,
rejection of old/truncated or oversized payloads through the LibXR packet
server, both event words and control degrees, 2027 event bits for red/blue,
reserved ally slots, and the unknown-energy strategy behavior. Physical
UART/CAN3 captures and the basic stance 1/2/3 loop still require the paired
MC02 hardware acceptance described in firmware issue #30.

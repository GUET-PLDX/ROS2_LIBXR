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

The ROS Behavior message is applied before the world-to-chassis rotation. An active
`tunnel_escape_active` replaces navigation XY with the world-frame escape XY.
Then, when `tunnel_prepare_active && !tunnel_ready`, both selected XY components
are multiplied by 0.15. This matches the original communication node, including
slowing escape motion while preparation is incomplete. Readiness or leaving the
preparation phase restores unscaled input on the next send cycle; the cached
navigation velocity is never scaled in place.

These operations do not change `wz`, the alignment target, or firmware gyro-mode
selection. Preparation and readiness stay on the ROS side, so the wire ABI is
unchanged. The original navigation stream must still be valid and fresh before
an override is allowed; selected escape values and odometry also pass the existing
validity checks. Behavior state is retained until the next Behavior message, as
in the original node; this change adds no behavior-heartbeat policy.

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

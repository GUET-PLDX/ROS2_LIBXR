# ROS2_LIBXR navigation bridge

This package adapts the navigation workspace to the MC02 firmware SharedTopic
protocol. It is built around the upstream `ROS2_LIBXR` repository and retains
its LibXR transport implementation.

## Topics

The bridge publishes `nav_data` and `behavior_data` at 50 Hz over USB CDC and
receives `team_info`, `game_info`, `online_info`, `offline_info`, and
`radar_info`. It subscribes to `/cmd_vel_mpc`, `/chassis_cmd_mpc`, and
`/sentry/behaivor_send`, and publishes the matching `/sentry/*` status topics
consumed by BT Manager.

By default, command components are forwarded unchanged. A fresh
`/chassis_cmd_mpc` takes priority and forwards velocity and force feedforward
from the same controller cycle. After 150 ms without that command, the bridge
uses a fresh `/cmd_vel_mpc` with zero force; if both streams are stale, it sends
zero heartbeats. The optional `rotate_world_to_body` parameter rotates velocity
and force together using `/aft_mapped_to_init`; stale odometry stops output in
that mode. Behavior scan and tunnel alignment angles are converted from degrees
to radians. Firmware-only fields in the wire protocol are zeroed.

## Run

`start.bash` starts the bridge by default. Set `LIBXR_BRIDGE_ENABLE=0` to run
without the physical USB link. For standalone launch:

```bash
ros2 launch ros2_libxr ros2_libxr_launch.py
```

Parameters include `cmd_topic`, `behavior_topic`, `baudrate` (921600),
`usb_vid` (`16d0`), `usb_pid` (`1492`), and `control_interface`
(`XRUSB Navigation Control`).

## Wheel odometry

The bridge also receives `chassis_feedback` (`wire::ChassisFeedback`, 26 bytes).
It mirrors `Pldx::DualBoardControl::ChassisFeedback` in the MC02 `DualBoard`
module. The chassis board computes the body-frame velocity from the four omni
wheels every 1 ms and sends it to the gimbal board on CAN `0x32B` at 250 Hz. The
gimbal board adds an extended sequence number, a receive timestamp and the latest
chassis gyro z, then forwards it on the navigation CDC.

The bridge publishes it as `geometry_msgs/TwistStamped` on
`chassis_velocity_topic` (default `/sentry/chassis_velocity`, frame `base_link`,
sensor-data QoS). `FollowPath.dynamic.wheel_odometry` subscribes to that topic.

- **Timestamp:** `BoardClockMapper` maps the board's 32-bit µs counter to ROS time
  with a running minimum of the receipt offset, so USB jitter does not reach the
  stamp. A reset or a jump restarts the mapping. `chassis_velocity_latency_us`
  (default 1000) is then subtracted for the chassis sampling and CAN transfer.
- **Dropped samples:** invalid, link-down and duplicate samples are not published,
  so the controller falls back to LIO. By default, samples with any wheel offline
  are dropped too (`chassis_velocity_require_all_wheels`). Sequence gaps are
  logged.
- **Yaw rate:** `chassis_yaw_rate_source` is `wheel` (default) or `gyro`. `gyro`
  uses the chassis IMU when it is valid and otherwise falls back to the wheels.
  The wheels slip at high spin rates.

Before enabling `wheel_odometry` on the robot, drive with translation and rotation
while running:

```bash
python3 scripts/check_chassis_velocity_frame.py --seconds 60
```

It fits the wheel velocity against the lever-arm-compensated LIO velocity and
reports the 2×2 translation matrix, rotation, mirroring and the yaw-rate scale and
sign. It exits nonzero if they differ from identity.

## Protocol caveats

The firmware's enemy position fields are unsigned 16-bit values with no unit
declared in the current wire definition. They are copied numerically into ROS
position fields; consumers must not treat them as meters until firmware defines
the scale. The current firmware also does not transmit several fields available
in the ROS Behavior and offline status messages.

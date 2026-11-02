# ROS2_LIBXR navigation bridge

This package adapts the navigation workspace to the MC02 firmware SharedTopic
protocol. It is built around the upstream `ROS2_LIBXR` repository and retains
its LibXR transport implementation.

## Topics

The bridge subscribes to `/chassis_cmd_mpc` and `/sentry/behaivor_send` and
publishes `nav_data` and `behavior_data` at 100 Hz over navigation USB CDC.
`nav_data` is three packed little-endian float32 values: chassis-frame `Fx`, `Fy`,
`Mz` (12 bytes, N/N*m). Both MC02 boards must use the matching firmware.

The command's source stamp and steady-clock receipt age must be within 150 ms.
A missing, stale, non-finite or non-planar command produces zero wrench heartbeats.
Current LIO odometry is required for the world-to-chassis rotation. Its frame must
match the command frame. `chassis_yaw_offset` (rad) calibrates the LIO-to-chassis
axis offset; the heading is projected over odometry age plus `rotate_latency`.
The chassis control point and axes must match the firmware force allocation.

The bridge receives `team_info`, `game_info`, `online_info`, `offline_info`,
and `radar_info`, and publishes the corresponding ROS status topics. Behavior angles are converted to radians.

## Run

`start.bash` starts the bridge by default. Set `LIBXR_BRIDGE_ENABLE=0` to run
without the physical USB link. For standalone launch:

```bash
ros2 launch ros2_libxr ros2_libxr_launch.py
```

Parameters include `chassis_cmd_topic`, `chassis_yaw_offset`, `behavior_topic`, `baudrate` (921600),
`usb_vid` (`16d0`), `usb_pid` (`1492`), and `control_interface`
(`XRUSB Navigation Control`).

## Protocol caveats

The firmware's enemy position fields are unsigned 16-bit values with no unit
declared in the current wire definition. They are copied numerically into ROS
position fields; consumers must not treat them as meters until firmware defines
the scale. The current firmware also does not transmit several fields available
in the ROS Behavior and offline status messages.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 colcon workspace for a hoverboard-chassis differential-drive robot. An ESP32-S3 talks to the hoverboard board (EFeru hoverboard-firmware-hack-FOC) over UART, reads a robot-vacuum LiDAR and an MPU6050, and streams everything to ROS over Wi-Fi. slam_toolbox builds the map. A Gazebo Sim model (`hoverbot_sim`) exposes the same topics and frames as the real robot, so SLAM settings can be tested in simulation. [README.md](README.md) has the full file map, node parameters, install steps and the odometry calibration procedure.

The README targets Jazzy, but this machine has ROS 2 **Kilted** + Gazebo Sim 9 (`/opt/ros/kilted`). The ROS nodes are C++ (rclcpp); launch files are Python. Code comments and docs are in Russian; keep that style.

## Commands

```bash
source /opt/ros/kilted/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test --packages-select esp32_bridge && colcon test-result --verbose   # gtest: protocol
./build/esp32_bridge/test_protocol --gtest_filter='Scan.*'                   # single test

ros2 launch hoverbot_sim mapping.launch.py [gui:=false rviz:=false]   # Gazebo + RViz + slam_toolbox
ros2 launch esp32_bridge robot.launch.py transport:=udp host:=<ESP IP>  # real robot (or transport:=serial)
ros2 launch hoverbot_nav sim_navigation.launch.py [gui:=false rviz:=false]            # sim + SLAM + Nav2 + octomap
ros2 launch hoverbot_nav robot_navigation.launch.py transport:=udp host:=<ESP IP>    # real robot + Nav2 (robot moves!)
cd firmware/esp32_bridge && pio run -e wifi -e usb                   # build both ESP32 firmwares
                                                                    # (pio: ~/.platformio/penv/bin/pio)
git submodule update --init   # firmware/hoverboard-firmware-hack-FOC
```

**Hanging ROS CLI / launch:** after processes are killed with `kill -9`, stale Fast DDS shared-memory segments make `ros2 topic echo`, `ros2 node list`, rclpy nodes and launch files hang silently. Run `fastdds shm clean`, or `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Use `timeout -s KILL` in test scripts, because hung rclpy processes ignore SIGTERM. Don't `pkill -f` a pattern that also appears in your own shell command line, or you'll kill your own shell.

## Architecture

- **ESP32-S3 firmware** `firmware/esp32_bridge/` (PlatformIO, not colcon). One codebase builds two firmwares: `pio run -e wifi` (UDP) and `pio run -e usb` (native USB CDC, `/dev/ttyACM0`). `include/transport.h` picks the link with `-DTRANSPORT_WIFI` / `-DTRANSPORT_USB`; `src/main.cpp` doesn't depend on the transport.
  - The firmware parses hoverboard `SerialFeedback` frames byte by byte (0xABCD start, XOR checksum; speeds are **rpm**, converted with `RPM_TO_MPS`) and integrates wheel odometry.
  - The LiDAR runs in a FreeRTOS task that sends scan chunks of 8 frames.
  - Wi-Fi credentials go in the gitignored `include/secrets.h`.
- **Wire protocol v2** (spec at the top of `firmware/esp32_bridge/include/protocol.h`):
  - Packet: `[type u8][seq u16][stamp u64][payload][crc16 Modbus]`. UDP sends one datagram per packet; USB uses COBS framing with `0x00` delimiters.
  - Types ESP→PC: ODOM 0x01, SCAN chunk 0x02, PONG 0x03, LOG 0x04. Types PC→ESP: CMD 0x81, PING 0x82, RESET_ODOM 0x83.
  - The PC-side mirror is `src/esp32_bridge/include/esp32_bridge/protocol.hpp` + `src/protocol.cpp`. **Change both sides together.**
  - `test/test_protocol.cpp` includes the firmware's `protocol.h` and cross-checks the two implementations.
- **esp32_bridge node** (`src/bridge_node.cpp`, C++):
  - The `transport` param (`udp` | `serial`) selects `UdpTransport` or `SerialTransport`. Both run on Boost.Asio on their own `io_context` thread; ROS callbacks hand outgoing packets over with `asio::post`.
  - PING runs at 2 Hz and feeds `TimeSync`, which keeps the min-RTT offset. Every stamp is ESP time converted to PC time.
  - `ScanAssembler` rebuilds revolutions from chunks. `bin_scan` puts points into fixed angular bins by their true angle.
  - The last `/cmd_vel` is resent at 20 Hz for `cmd_timeout`. The firmware stops the wheels after 300 ms with no commands.
  - Service `/reset_odom`. Link statistics are logged every 10 s.
  - `publish_map_odom_tf` is off by default; a static `map→odom` conflicts with slam_toolbox and breaks loop closure.
  - `launch/robot.launch.py` starts the node, slam_toolbox and RViz for the real robot.
- **Values that must stay in sync** (README geometry table):
  - wheel base: `WHEEL_BASE` (`config.h`) = `wheel_base` param = `wheel_separation` (xacro) = 0.54 m;
  - wheel diameter: `WHEEL_DIAMETER` = 2 × `wheel_radius` = 0.16 m;
  - `HB_BAUD` (`config.h`) = `USART*_BAUD` in the FOC firmware `Inc/config.h`.
- **Baud caveat:** the FOC submodule defaults to `USART2_BAUD`/`USART3_BAUD` = 115200 with no local edits, while `HB_BAUD` = 1000000. Set the same value on both sides (e.g. `-DUSART2_BAUD=1000000` in the `VARIANT_USART` build flags).
- **Pin caveat:** the pins in `config.h` were copied from the old firmware (LiDAR 32/33) and look wrong for an ESP32-S3: GPIO 26–37 are flash/PSRAM on most modules. IMU moved to GPIO 8/9 (old 21/22 never worked: S3 has no GPIO 22). Wiring diagram: `firmware/esp32_bridge/PINOUT.md`.
- **Simulation** `src/hoverbot_sim`:
  - All robot dimensions live in the parameter block at the top of `urdf/hoverbot.urdf.xacro`. Front is the caster side; `base_link` is at the wheel-axle centre.
  - Uses gz DiffDrive (publishes odom + TF `odom→base_link`), a `gpu_lidar` with `gz_frame_id` = `laser`, and an IMU. Topics are bridged by `config/bridge.yaml`.
  - Gazebo DiffDrive keeps executing the last `/cmd_vel` until a new one arrives.
- **`src/twist_converter`**: standalone node `Twist → TwistStamped` (params `input_topic`, `output_topic`, `frame_id`) for ros2_control's `diff_drive_controller`, which in Jazzy+ accepts only `TwistStamped`. It is not used by the current launch files (bridge and gz DiffDrive take plain `Twist`).
- **`src/hoverbot_nav`** (Nav2 + octomap, shared by sim and real robot; docs in `docs/NAV2_OCTOMAP.md`, status in `docs/CHECKLIST.md`):
  - `navigation.launch.py` runs Nav2 nodes (controller/planner/smoother/behaviors/bt_navigator/velocity_smoother) + `scan_to_cloud` + `octomap_server` on top of an already-running SLAM. Wrappers: `sim_navigation.launch.py` (Gazebo + SLAM + Nav2) and `robot_navigation.launch.py` (esp32_bridge + SLAM + Nav2, `use_sim_time:=false`).
  - No `map_server`/`amcl`: slam_toolbox owns `/map` and `map→odom`. Nav2 publishes plain `Twist` (`enable_stamped_cmd_vel: false`) via `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel`.
  - `octomap_server` must keep `latch: True` (transient_local), otherwise the costmap static layer never receives `/projected_map`. Octomap is fed from the 2D scan, so it is a single voxel layer.
  - Costmap `footprint` (both costmaps in `nav2_params.yaml`) must match the URDF dimensions.
- **slam_toolbox is a lifecycle node** (Jazzy+). Start it via its own `online_async_launch.py`, as `mapping.launch.py` does; a plain `Node` stays unconfigured and `map` never appears. That launch file defaults to `use_sim_time:=true`, so pass `use_sim_time:=false` on the real robot.
- SLAM params: `hoverbot_sim/config/slam_sim.yaml` and `esp32_bridge/config/slam_param.yaml` are identical except `use_sim_time`; change them together.

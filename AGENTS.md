# AGENTS.md

ROS 2 Humble (Ubuntu 22.04) workspace for the AIC Car 2025 competition. C++ (`ros2_tools`, `robot_gazebo`), Python (`robot_real`, `vision_node`, `yolip`, `tts`), Rust (`navi_rs`, rclrs/ament_cargo). The competition is normally built and run on a Jetson; this checkout is a dev copy with no `build/`, `install/`, or `log/`.

## Build / verify

- `make build` = `colcon build --packages-select ros2_tools robot_real vision_node tts yolip navi_rs`. `ros2_tools` defines all msg/srv and must be built first.
- `robot_gazebo` is **not** in `make build`; build it separately (`colcon build --packages-select robot_gazebo`).
- `make clean` removes `build/ install/ log/`.
- `navi_rs/Cargo.toml` path-depends on absolute `/home/jetson/utils/ros2_rust/install/...` plus `../install/ros2_tools/share/ros2_tools/rust`. Plain `cargo build`/`cargo test` only works after `ros2_tools` is installed and on a host with that exact prefix. CI (`.github/workflows/rust.yml`) is `workflow_dispatch` only.
- No lint/typecheck target. Tests are thin: `navi_rs/tests/test_navigation.sh` is a manual `ros2 topic pub/echo` helper, `vision_node/test_vision_node.py` is a manual service client, `test/` dirs are only ament lint (copyright/flake8/pep257). Don't assume `colcon test` covers behavior.

## Packages / entrypoints

- `ros2_tools` (ament_cmake): nodes `lidar_data_node`, `bsp_node`, `camera_node`, `hardware_bridge_node`, `pwm_node` (Python); defines `msg/LidarPose` and srv `YOLO`, `OCR`, `SERVO`, `TTS`, `GarbageClassify`. Only package with custom interfaces.
- `navi_rs` (ament_cargo): the whole mission sequence (waypoints, servo, vision triggers, TTS) is hardcoded in `navi_rs/src/main.rs`; publishes `/goal`, subscribes `/lidar_data`, blocks on the services below.
- `robot_gazebo`: Gazebo classic sim; `libodom_plugin.so` publishes `/absolute_pose`.
- `robot_real`: launch files + `robot_real` simple goal publisher only.
- `vision_node`: executables `vision_gazebo` / `vision_real`. (`vision_node/README.md` says `ros2 run vision_node node` — wrong.)
- `yolip`: YOLO/CLIP garbage classifier, service `garbage_classify`, subscribes `/camera/video`.
- `tts`: Kokoro TTS, service `tts_play`.

## Runtime wiring

- `navi_rs` → `/goal` (PoseStamped, frame `odom`) → `bsp_node` (position/yaw PID) → `/cmd_vel` → `hardware_bridge_node` (real) or Gazebo planar_move (sim).
- `lidar_data_node` converts odom → `/lidar_data`. It subscribes to **both** `/absolute_pose` (sim) and `/aft_mapped_to_init` (real) regardless of `use_simulation`.
- Services: `yolo_trigger`/`ocr_trigger` (vision_node), `garbage_classify` (yolip), `servo_control` (pwm_node), `tts_play` (tts).
- Cameras: sim `/camera1/image_raw`, `/camera2/image_raw`; real `/camera/video` (mono), `/camera/d435/color/image_raw` (D435). `navi_rs` always asks for `camera1`/`camera2`; `vision_real` maps them internally.

## Launch

- Sim: `ros2 launch robot_gazebo robot.launch.py`; `ros2 launch ros2_tools tools_gazebo.launch.py`; `ros2 run vision_node vision_gazebo`; `ros2 run navi_rs navi_rs`.
- Real: `ros2 launch robot_real tools.launch.py` (= slam + `ros2_tools tools_real.launch.py`); `robot_real/launch/vision.launch.py` starts `vision_real` + `yolip`; then `ros2 run navi_rs navi_rs`.
- `make launch` is stale: it starts `ros2 launch robot_real all.launch.py`, which does not exist (tmux session `my_grid_env`; `make stop` kills it).

## Gotchas

- README's "Serial Protocol" section is wrong. Actual `hardware_bridge_node`: header `AA BB 0A 12 02`, then Vx/Vy/Omega as 3× int16 little-endian (`round(v*1000)`), then `0x00`, sent at 50 Hz; startup sequence `0x11` + nine `0x00`; watchdog actually stops motors after 5 s.
- Model weights are `.gitignore`d and absent: `vision_node/yolo/*.pt` (YOLO service model names map to `<name>.pt`), `vision_node/ocr/` (PaddleOCR, removed from git; override with ROS param `vision_node_src_dir`), `yolip/yolip/scripts/rubbish.pt`, `yolo_cls.pt`, `cn_clip/`. `vision_real.py:73` also hardcodes `/home/jetson/ros2/AIC_car_2025/vision_node/yolo/result`.
- `robot_gazebo/launch/robot.launch.py` creates a `mecanum_controller` spawner but never adds it to the returned `LaunchDescription` (and hardcodes a `/home/dev/...` yaml path); spawn the controller manually, see `robot_gazebo/cmd.txt`.
- `ros2_tools/src/camera_node.cpp:68` currently ignores the physical camera and publishes the static image `/home/jetson/ros2/AIC_car_2025/vision_node/yolo/1.jpg`; the real capture is commented out.
- `vision_gazebo.py` reads `config.YOLO_LABELS`, which `config.py` never defines; `vision_real` is the node used by `robot_real/launch/vision.launch.py`.
- `ros2_tools/src/topic_to_gazebo.cpp` is not referenced by `CMakeLists.txt` and is not built.
- `pwm_node` imports `Jetson.GPIO` (BOARD pin 13, 50 Hz, command ∈ {-1,0,1}) — real-hardware only.
- `yolip` hardcodes `/home/jetson/ros2/AIC_car_2025/yolip/yolip` for its YAML/CLIP/weight paths.

## Conventions

- Comments, logs, and TTS text are in Chinese; keep new user-facing strings Chinese.
- Add new interfaces only to `ros2_tools` (`msg/` or `srv/` in `CMakeLists.txt`); consumers use `ros2_tools.msg/srv` (Python), `ros2_tools::msg`/`::srv` (C++/Rust).

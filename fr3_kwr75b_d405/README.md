# FR3 with KWR75B

FR3 demo with a Kunwei KWR75B force/torque sensor, a 6 mm ISO 9409-1-50-4 adapter, a Franka Hand and a D405 holder.
The sensor geometry and acquisition launch come from `kunwei_ft_driver`.

## Install

Build from the workspace root after installing the ROS dependencies:

```bash
source /opt/ros/humble/setup.bash
cd ~/local_ws
rosdep install --from-paths src/compliant_controllers_demos/fr3_kwr75b_d405 --ignore-src -r -y
colcon build --packages-up-to fr3_kwr75b_d405 --symlink-install
source install/setup.bash
```

## Run

Preview the assembly without hardware connections:

```bash
ros2 launch fr3_kwr75b_d405 bringup.launch.py mode:=view start_sensor:=false start_camera:=false
```

Real mode is the default, with robot IP `192.168.1.1`. The description anchors the robot base to `world`.

Start the real robot, Hand and sensor through the shared `fr3_real` bringup:

```bash
ros2 launch fr3_kwr75b_d405 bringup.launch.py mode:=real robot_ip:=192.168.1.1
```

Use `start_hand:=false` or `start_sensor:=false` to disable an accessory driver.
The RealSense D405 driver starts by default. Select another model with:

```bash
ros2 launch fr3_kwr75b_d405 bringup.launch.py camera_model:=D435I
```

Use `start_camera:=false` to disable the camera; `start_camera:=auto` enables it only in real mode.
`camera_model` defaults to `D405` and selects the driver's `device_type` (for example,
`D435I` or `D455`). Set `camera_serial_no:=123456789012` to select a particular unit;
the launch adds the driver's required underscore prefix automatically.
The node and topic prefix default to the lowercase model: `/d405` and
`/d405/color/image_raw`. RViz includes a color image view that follows this prefix.
Use `camera_namespace` and `camera_name` to override it. Color streaming is enabled.
This includes the upstream `realsense2_camera/rs_launch.py`; install `realsense2_camera`
through rosdep before enabling it. Camera selection changes the driver only; the
assembly geometry remains the D405 holder. A static transform connects `d405_holder` to the driver root (`d405_link` by
default), including when `show_camera:=false`. The driver publishes the internal
camera transforms. `camera_mount_xyz` and `camera_mount_rpy` set the root pose
relative to the holder. Their defaults match the approximate CAD envelope; replace
them with calibrated extrinsics for measurements in the robot frame. Other camera
models also need their mounting pose adjusted.

Use `sensor_host`, `sensor_port` and `sensor_prefix` to configure acquisition; the default prefix is `ft_sensor`.
The prefix sets the sensor frame, wrench topics and software-zero service.
The extra visualization stream is enabled only when `rviz:=true`.
Use `rviz:=false` for bringup without a visualization window.
The sensor starts in both modes by default; `start_sensor:=auto` enables it only in real mode.
View mode uses namespace `/fr3_kwr75b_d405` and TF prefix `fr3_kwr75b_d405_view/`; real mode uses the shared `/fr3` bringup.
The default controller frame is `fr3_link8`; controller and friction-compensation options are passed to `fr3_real`.

## Assembly

The sensor measurement frame is 37.5 mm from the flange, with native yaw 300° and Hand yaw −15°.
Use `show_fasteners:=true` to show illustrative screw paths and thread-overrun markers.
Use `show_camera:=false` to hide the approximate camera envelope independently of `start_camera`.
The holder registration can be adjusted with `holder_xyz` and `holder_rpy`.
The D405 optical preview frame is illustrative and is not a camera calibration.
The holder fit and cable clearance require physical verification; added tool inertias are not calibrated.

Startup software zeroing is optional: pass `zero_sensor:=true` (default: false).
It waits for the zero service and fresh samples, retries until successful, then exits.
Use `zero_timeout:=30.0` to set the wait limit in seconds; failure is logged and bringup continues.
The current load becomes the zero baseline, so keep the tool unloaded and stationary.
This is a software offset lasting until sensor driver restart; controllers start concurrently.

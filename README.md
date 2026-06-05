# compliant_controllers_demos

Robot-specific launch and configuration demos for `compliant_controllers`.

This package contains FR3/Panda, FR3 Gazebo, UR Gazebo, and LBR/IIWA Gazebo launch files plus the controller YAML and profile files needed by those demos.

Main project: https://github.com/smihael/compliant_controllers

## Examples

```bash
ros2 launch compliant_controllers_demos fr3_gz.launch.py load_gripper:=false
ros2 launch compliant_controllers_demos fr3.launch.py controller_name:=cartesian_impedance_controller
ros2 launch compliant_controllers_demos fr3_robotiq.launch.py com_port:=/dev/ttyUSB0
ros2 launch compliant_controllers_demos fr3_spacemouse_teleop.launch.py load_gripper:=false
ros2 launch compliant_controllers_demos ur_gz.launch.py ur_type:=ur10e
ros2 launch compliant_controllers_demos lbr_gazebo.launch.py model:=iiwa14 ctrl:=cartesian_impedance_controller
```

The SpaceMouse teleop demo starts the FR3 controller, the existing `spacemouse_publisher`
node, and a bridge that converts SpaceMouse `Twist` messages into stamped
`compliant_controllers_msgs/CartesianCommand` targets.

The FR3 Robotiq demo starts the FR3 compliant controller launch, the Robotiq
2F-85 control launch, and RViz with a composed FR3 + Robotiq description.

## Build

From the workspace root:

```bash
source install/setup.bash
colcon build --packages-select compliant_controllers_demos --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --symlink-install
```

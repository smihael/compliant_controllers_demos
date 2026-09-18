# LBR Gazebo (FRI 1.15)

LBR Gazebo demo using the `lbr_fri_ros2_stack` Humble
branch and FRI client version 1.15. The FRI client is required to build the
LBR stack; Gazebo itself runs in simulation and does not connect to a physical
KUKA controller.

## Docker

Build the shared controller image once from the `compliant_controllers_demos`
repository root:

```bash
docker build . -t compliant-controllers:humble-desktop-full
cd lbr_gz_sim
docker build . -t compliant-controllers:lbr-gz-sim
```

Run from this directory:

```bash
xhost +si:localuser:root
docker compose up --build
```

The default model is `iiwa14` and the default controller is
`cartesian_impedance_controller`. For headless execution:

```bash
SHOW_GAZEBO_GUI=false docker compose up --build
```

Set `SHOW_RVIZ=true` to enable RViz, or `ROS_DOMAIN_ID` to override the default
of 2. `LBR_BASE_IMAGE` overrides the shared image for Compose builds;
for a direct build use `--build-arg BASE_IMAGE=...`. `FRI_CLIENT_VERSION`
defaults to 1.15 and can be overridden as an environment variable for Compose
or a build argument for Docker.

Stop the demo with `docker compose down`.
GUI forwarding uses the host's X11 display.

## Send a Cartesian step

Start the simulation and send one **5 mm step along base-frame X** as soon as its
controllers are healthy:

```bash
docker compose --profile demo up --build
```

For an already-running simulation, run the step on demand:

```bash
docker compose run --rm demo
```

Each invocation sends one step relative to the current pose and exits. Repeat
the command to send another step. The sender times out after 30 seconds if it
cannot finish. Plain `docker compose up` starts only the simulation.

Use the same `ROS_DOMAIN_ID` for the simulation and command sender.

```bash
docker compose logs demo
docker compose --profile demo down
```

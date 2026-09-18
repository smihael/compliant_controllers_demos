# Flexiv Rizon 4s in Gazebo

ROS 2 Humble / Gazebo Fortress Docker demo using the shared Cartesian
impedance controller, gravity compensation, seven effort interfaces, and full
Flexiv visual meshes.

## Build and run

Requires Docker Engine and Compose v2. From the `compliant_controllers_demos` repository root:

```bash
# Skip if already built:
docker build -t compliant-controllers:humble-desktop-full .
cd flexiv_gz_sim
docker compose up --build -d --wait
docker compose logs --no-color -f flexiv-gazebo
```

From another terminal in this demo directory:

```bash
# Send a relative 5 mm base-frame X step:
docker compose run --rm --no-deps demo
# Verify pose stability and measure another 5 mm step:
docker compose run --rm --no-deps smoke-test
# Stop:
docker compose down
```

`smoke-test` checks pose stability and motion, returning nonzero on failure.
Start with `up --wait` before using `--no-deps`, and run with no other command
senders. For a one-shot headless test:

```bash
docker compose --profile test up --build --abort-on-container-exit --exit-code-from smoke-test
docker compose down
```

## X11 viewer

With X11/XWayland and `DISPLAY` set:

```bash
xhost +si:localuser:root
docker compose -f docker-compose.yml -f docker-compose.x11.yml up --build -d --wait
docker compose run --rm --no-deps demo
```

```bash
docker compose down
xhost -si:localuser:root
```

## Configuration

Controller settings are in [`config/rizon4s_controllers.yaml`](config/rizon4s_controllers.yaml).
The Cartesian frames are `base_link` and `flange`.

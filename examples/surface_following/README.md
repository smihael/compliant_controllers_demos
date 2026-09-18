# Surface-following example

The simulator extends the locally built `compliant-controllers:franka-gz-sim`
image. Build it first using the [Franka Gazebo instructions](../../franka_gz_sim/#docker).
The entire example is mounted at `/opt/demo` in both containers.
Edit notebooks, scripts, fixtures, URDF and startup scripts without rebuilding.

From this directory:

```bash
xhost +si:localuser:root
docker compose up --build -d
```

Defaults: Gazebo GUI on, **ROS domain 1** (shared with Franka Gazebo), **Jupyter port 8889**, token `surface`.
Open http://localhost:8889/lab?token=surface and select
`notebooks/surface_following.ipynb`.

Choose a port and domain using environment variables or a local `.env` file:

```bash
JUPYTER_PORT=8890 ROS_DOMAIN_ID=1 docker compose up -d
```

Host networking supports ROS discovery. `JUPYTER_PORT` sets the actual host
listening port; an occupied port causes a clear failure instead of silently
selecting another. Set `JUPYTER_TOKEN` to change the token.

The camera and optional viewer run inside the Gazebo container:

```bash
ENABLE_SURFACE_CAMERA=true SHOW_CAMERA_VIEWER=true docker compose up -d
```

For headless simulation set `SHOW_GAZEBO_GUI=false` and leave the viewer off.

Run the full demo from the mounted source:

```bash
docker compose exec notebook bash /opt/demo/docker/entrypoint.sh \
  python /opt/demo/scripts/surface_demo.py \
  --execute --travel 0.18 --output /opt/demo/notebooks/surface_run.npz
```

```bash
docker compose ps
docker compose logs notebook
docker compose down
```

Rebuild after changing dependencies or the Franka launch package.

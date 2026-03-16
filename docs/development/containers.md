# containers

This repo now has a first-pass Docker setup for simulation work. The aim is modest on purpose: get a repeatable Linux ROS 2 and Gazebo environment that can run on a cloud GPU VM without rebuilding the machine from scratch every time.

The first image is `sim-dev`. It is meant for `amd64` Linux boxes with an NVIDIA GPU and the NVIDIA container runtime. It is not the final competition image, and it is not the Jetson runtime image. It is the development box we use to build the workspace, run Gazebo headless, and keep the environment sane while the sim and rover are still changing.

`rocker` still has a place here, but not as a substitute for Dockerfiles. Think of it as a nicer launcher for interactive Linux sessions once the image exists. The image is the recipe. `rocker` is just a convenient way of running it.

## what is in the first pass

The Docker scaffold currently includes:

- `docker/Dockerfile` with a `sim-dev` target
- `docker/scripts/build_sim_image.sh`
- `docker/scripts/run_sim_dev.sh`
- `docker/scripts/run_sim_launch.sh`
- `docker/scripts/run_navigation_launch.sh`
- `docker/scripts/run_foxglove_bridge.sh`
- `.dockerignore`

The image installs:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress with `ros_gz`
- common ROS development tools
- repo dependencies via `rosdep`
- `open3d` for the current perception pipeline

It also creates a non-root `ros` user so bind-mounted files do not end up owned by root.

## build the image

From the repo root:

```bash
./docker/scripts/build_sim_image.sh
```

That builds the `sim-dev` target for `linux/amd64` and tags it as `innex1/sim-dev:local`.
By default it uses plain `docker build`, because that is the least awkward path on team Linux boxes and cloud VMs.

If Docker still needs `sudo` on your machine, that is fine. The helper scripts will fall back to `sudo docker` automatically when plain `docker` is not available to your user. The build script still uses the real repo ownership on disk so the container user lines up with the host user instead of creating root-owned files by accident.

If you want a different tag:

```bash
IMAGE_NAME=innex1/sim-dev:2026-03-16 ./docker/scripts/build_sim_image.sh
```

If you need to override the platform explicitly:

```bash
PLATFORM=linux/amd64 ./docker/scripts/build_sim_image.sh
```

If you really do want `buildx`, for example on a cross-build machine, turn it on explicitly:

```bash
USE_BUILDX=true ./docker/scripts/build_sim_image.sh
```

## run a shell in the container

On a Linux machine with NVIDIA container support:

```bash
./docker/scripts/run_sim_dev.sh
```

That starts an interactive shell with:

- the repo bind-mounted at `/workspaces/innex1-rover`
- `--gpus all`
- host networking
- host IPC

Inside the container, build the workspace in the mounted repo:

```bash
colcon build --symlink-install
source install/setup.bash
```

Then launch the sim:

```bash
ros2 launch lunabot_simulation moon_yard.launch.py
```

If you prefer one-command helpers instead of an interactive shell, use the scripts below from the repo root:

```bash
./docker/scripts/run_sim_launch.sh
./docker/scripts/run_navigation_launch.sh
./docker/scripts/run_foxglove_bridge.sh
```

These all use host networking, host IPC, the mounted repo, and the same `ROS_DOMAIN_ID`.

The launcher scripts also create predictable container names:

- `innex1-sim`
- `innex1-navigation`
- `innex1-foxglove`
- `innex1-sim-dev`

That makes it much easier to inspect logs and stop things cleanly.

## recommended team workflow

For a shared team setup, this should be the default path on Linux VMs and cloud machines.

In terminal 1, build the image once:

```bash
./docker/scripts/build_sim_image.sh
```

or, if your Docker daemon still needs it:

```bash
sudo ./docker/scripts/build_sim_image.sh
```

In terminal 2, build the workspace inside the image:

```bash
./docker/scripts/run_sim_dev.sh
colcon build --symlink-install
source install/setup.bash
exit
```

After that, day-to-day use is just three terminals from the repo root:

```bash
./docker/scripts/run_sim_launch.sh
./docker/scripts/run_navigation_launch.sh
./docker/scripts/run_foxglove_bridge.sh
```

If you are on a remote VM and want the stack left running in the background, use detached mode:

```bash
DETACH=true ./docker/scripts/run_sim_launch.sh
DETACH=true ./docker/scripts/run_navigation_launch.sh
DETACH=true ./docker/scripts/run_foxglove_bridge.sh
```

You do not need to prefix the scripts with `sudo`. If your user is not in the Docker group yet, the helper layer falls back to `sudo docker` on its own.

Then open Foxglove in the browser and connect to:

```text
ws://<vm-ip>:8765
```

If you are connecting from the same machine, use:

```text
ws://localhost:8765
```

If the VM firewall or AWS security group is not exposing the port, tunnel it over SSH instead:

```bash
ssh -L 8765:localhost:8765 -i /path/to/key.pem ubuntu@<vm-ip>
```

Then connect your local browser to:

```text
ws://localhost:8765
```

The scripts default to `ROS_DOMAIN_ID=0`. If the team needs a different domain on a shared machine:

```bash
ROS_DOMAIN_ID=7 ./docker/scripts/run_sim_launch.sh
ROS_DOMAIN_ID=7 ./docker/scripts/run_navigation_launch.sh
ROS_DOMAIN_ID=7 ./docker/scripts/run_foxglove_bridge.sh
```

Keep the same domain across every terminal.

To stop the named containers:

```bash
docker stop innex1-foxglove innex1-navigation innex1-sim
```

If Docker still needs `sudo`:

```bash
sudo docker stop innex1-foxglove innex1-navigation innex1-sim
```

To inspect their logs:

```bash
docker logs innex1-sim --tail 100
docker logs innex1-navigation --tail 100
docker logs innex1-foxglove --tail 100
```

If Docker still needs `sudo` on that host, use `sudo docker logs ...` instead.

## foxglove

Foxglove should be the default remote visualisation path for cloud and VM use.

It is a better fit than remote RViz2 for most day-to-day work because:

- it runs in the browser
- it is easy to share across machines
- it avoids full desktop OpenGL forwarding
- it is enough for topic inspection, transforms, images, point clouds, and debugging the nav stack

The bridge is already installed in the `sim-dev` image, and the helper script binds it to `0.0.0.0` on port `8765` so other machines can connect.

If you need a different port:

```bash
FOXGLOVE_PORT=9000 ./docker/scripts/run_foxglove_bridge.sh
```

Then connect to:

```text
ws://<vm-ip>:9000
```

## cloud GPU notes

This setup is aimed at cloud GPU VMs where desktop OpenGL forwarding is often the thing that goes wrong first.

If you are building from macOS, that is fine for the image itself. The actual GPU-backed sim run should still happen on a Linux machine or cloud VM with the NVIDIA container runtime available.

The intended path is:

1. install Docker
2. install the NVIDIA container toolkit on the VM
3. build the `sim-dev` image
4. run the container with `--gpus all`
5. run Gazebo server-side rather than depending on a forwarded desktop

Gazebo already has official headless rendering support through EGL, and the sim launch now exposes that through `headless_rendering:=true`.

In the container helper workflow, `docker/scripts/run_sim_launch.sh` defaults to:

```bash
HEADLESS_RENDERING=true
SERVER_ONLY=true
```

That is the sensible default for a Linux GPU VM where you mostly care about running the sim and inspecting it remotely through Foxglove.

If you want to drop both the EGL headless flag and the server-only flag, you can do that explicitly:

```bash
HEADLESS_RENDERING=false SERVER_ONLY=false ./docker/scripts/run_sim_launch.sh
```

That only changes Gazebo's launch mode. The current Docker helpers do not yet pass an X or Wayland desktop into the container, so the tested path is still headless Gazebo plus Foxglove.

If the machine already has Amazon DCV or another remote desktop, keep that as the interactive fallback for RViz2 and Gazebo GUI outside this first-pass container workflow. The main path should still be headless Gazebo plus Foxglove. It is less fragile and much easier to standardise across the team.

One honest caveat from testing on the AWS T4 path: the EGL warnings from Gazebo are not fatal, and point clouds come through fine. If you need camera-image debugging, treat that as a separate desktop-session problem for now rather than assuming the headless path will cover it.

## rocker

If you are on a Linux desktop and want an easier interactive session, `rocker` is worth trying once the image is built.

Example:

```bash
rocker --nvidia --network host --user --volume "$PWD:/workspaces/innex1-rover" innex1/sim-dev:local bash
```

I would not build the team workflow around `rocker` alone. Keep the Docker image and scripts as the baseline, then use `rocker` as a convenience layer where it helps.

## what this does not do yet

This is only the first pass. It does not yet:

- provide the Jetson `arm64` runtime image
- publish versioned images anywhere
- provide a polished container GUI path for RViz2 through DCV or X11
- split development and runtime images more aggressively
- guarantee every package builds on macOS through Docker Desktop

Those are the next steps once the basic sim image proves itself.

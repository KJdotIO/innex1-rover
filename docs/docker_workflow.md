# Docker Workflow

Use Docker for repeatable tooling, not as a way to pretend the whole rover is
inside a laptop.

The useful image is a rover development shell with ROS 2 Humble, colcon, rosdep,
vcstool, Python YAML support, Git and SSH. The source tree is mounted into the
container, so the image does not need to be rebuilt every time code changes.

## Local Dev Shell

Build the image:

```bash
docker build -f .devcontainer/Dockerfile -t innex1-rover-dev .
```

After the first main-branch publish, you can also pull the shared dev image:

```bash
docker pull ghcr.io/kjdotio/innex1-rover-dev:dev
```

Open a shell with the repo mounted:

```bash
docker run --rm -it \
  -v "$PWD:/workspace/innex1-rover" \
  -w /workspace/innex1-rover \
  innex1-rover-dev \
  bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
python3 tools/doctor.py --profile ci
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

Do not put SSH keys, Tailscale keys, router passwords or Jetson credentials in
the image. Mount secrets at runtime only when you deliberately need them.

## Publish Policy

Pull requests should build and smoke-test the dev image, but they should not
publish it.

Pushes to `main` should publish:

- `ghcr.io/<owner>/innex1-rover-dev:dev`
- `ghcr.io/<owner>/innex1-rover-dev:sha-<commit>`

Releases can publish extra semver tags later, but only when the team deliberately
cuts a release. For normal development, source is mounted into the container, so
the image changes only when the environment changes.

## What This Does Not Replace

Docker does not replace the Jetson, the router, sensor USB/Ethernet checks,
E-stop wiring, motor direction checks, fuse checks or any real motion test.

Use it for repo work, remote support, cheap tests and consistent contributor
setup. Use the actual rover for hardware truth.

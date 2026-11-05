# `docker/`: the caging simulator in a container

Runs the frozen simulator (tag `v1.0`, ROS 2 Jazzy + Gazebo Harmonic) without installing ROS 2:
only Docker is needed. Made for the students of the lecture; students who only want to run it
follow the [student guide](kit/STUDENT_GUIDE.md). This page is for building the image yourself and
for developing inside the container.

| File | Content |
|---|---|
| `Dockerfile` | image: `osrf/ros:jazzy-desktop` + dependencies (rosdep) + the five `caging_*` packages built |
| `entrypoint.sh` | loads ROS 2 and the workspace before any command |
| `env.sh` | `source docker/env.sh`: your user ID, screen, GPU and joystick groups for Compose |
| `compose.yaml` | services `test`, `experiments`, `sim`, `dev`, `gui` (Linux), `gui-nvidia` (Linux, NVIDIA GPU), `gui-wsl` (Windows) |
| `publish.sh` | author: build the student image (UID 1000, labels) and push it to `ghcr.io/ddrozo31/caging_multirobot_ros2` |
| `make_kit.sh` | author: `docker/dist/caging-docker-kit.zip` for the course page and the GitHub release |
| `kit/` | the kit's `compose.yaml` (pull-only, keep in sync with `compose.yaml`) and `STUDENT_GUIDE.md` |
| `../.dockerignore` | only `src/` and `docker/` go into the build |

## Requirements

- **Linux**: Docker Engine with the Compose plugin; your user in the `docker` group.
- **Windows 10 (21H2 or newer) or 11**: WSL2 with Ubuntu 24.04 and WSLg (`wsl --update`), and Docker:
  Docker Engine installed inside Ubuntu (recommended, it then works as on Linux) or Docker Desktop
  with the WSL2 back end and WSL integration for Ubuntu. Run every command in the Ubuntu terminal and
  clone the repository **inside** Ubuntu (`~/`, not `/mnt/c/...`): faster, and no Windows line
  endings in the scripts. Details: "Windows (WSL2 + WSLg)" below.
- About 9 GB of disk for the image (20 GB free recommended).
- **Processor x86-64 only** (the base `osrf/ros:jazzy-desktop` is amd64 only): ARM machines (Apple
  Silicon, Windows on ARM, Raspberry Pi) are not supported; arm64 support is possible later (multi-arch
  image from `ros:jazzy-ros-base`), not planned yet.
- **Hardware**: minimum 4 cores / 8 threads, H series or desktop CPU (2018+), 8 GB RAM on Linux or
  16 GB on Windows, hardware OpenGL 3.3+; recommended 6+ cores and 16 GB. The physics runs at 1 kHz, so
  CPU speed sets the real time factor: about 0.5 on an i7-9750H (headless), 0.13 or lower on a
  Surface Pro 7 (U series, WSL2, GUI). `experiments` needs about 0.2 or more to finish each run within
  the 4-minute limit of `run_experiments.sh` (frozen `src/`). Measured on that Surface Pro 7 (Core
  i5-1035G4, 8 GB, WSL2 with 3.6 GB), headless: 4 of 4 DONE, but each run took 184–233 s of the
  240 s limit, so it is at the edge. Table in the student guide.

## Build and test

```bash
cd caging_multirobot_ros2                # repository root
source docker/env.sh                     # once per terminal
docker compose -f docker/compose.yaml build              # first time: about 10-20 min
docker compose -f docker/compose.yaml run --rm test      # unit tests of caging_control
```

## Use

```bash
docker compose -f docker/compose.yaml run --rm sim       # shell with the simulator of the image
docker compose -f docker/compose.yaml run --rm dev       # shell with your src/ mounted
#   inside dev, after editing src/:  colcon build && source install/setup.bash
```

## Table 2 without a GUI

```bash
mkdir -p results/docker
docker compose -f docker/compose.yaml run --rm experiments   # about 8 min
```

The four box runs (static/tangent × waypoints/smooth) run headless; CSV files, plots and logs land in
`results/docker/` and the table is printed at the end. Expected (20261102): 4 of 4 runs DONE,
e_dO about 0.10 m with waypoints and 0.05 m with smooth trajectories, as in the frozen Table 2.

## Gazebo and RViz on Linux (X11)

Every terminal: `cd` to the repository root, `source docker/env.sh`, then
`docker compose -f docker/compose.yaml run --rm gui`. Each terminal is its own container; they share
the host network, so they see the same ROS 2 topics (same `ROS_DOMAIN_ID`, default 0).

Inside the first container, check the graphics: `glxinfo -B` must say `direct rendering: Yes` and
name your GPU (not `llvmpipe`, which is slow software rendering).

**Demo 1: drive one robot** (three terminals)

```bash
# terminal 1: Gazebo + RViz with one robot (wait until the robot appears)
ros2 launch caging_gazebo sim.launch.py
# terminal 2: joystick + twist_mux (twist_mux forwards keyboard AND joystick to the robot)
ros2 launch caging_bringup teleop.launch.py
#   joystick: hold LB (dead-man button), left stick = forward/sideways, right stick = rotate; RB = turbo
# terminal 3: keyboard (keep this terminal focused while typing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key
#   i / , forward / back, j / l rotate, J / L (Shift) sideways, k stop
```

Without terminal 2 nothing moves: the keyboard publishes on `cmd_vel_key` and only `twist_mux` passes
it to `mecanum_controller/cmd_vel`. The joystick wins over the keyboard while LB is held.

**Demo 2 and 3** (one terminal each, the previous demo stopped with Ctrl+C):

```bash
ros2 launch caging_bringup caging.launch.py                    # Demo 2: approach and closure (A*)
ros2 launch caging_bringup task.launch.py auto_start:=true     # Demo 3: the whole task
```

**If the robot does not move**, check in an extra `gui` terminal, one link at a time:

```bash
ros2 run joy joy_enumerate_devices            # the joystick is listed (else: plugged in? env.sh sourced?)
ros2 topic echo /joy                          # numbers change when you move the sticks
ros2 topic echo /cmd_vel_joy                  # non-zero only while LB is held
ros2 topic echo /cmd_vel_key                  # changes when you type in the keyboard terminal
ros2 topic echo /mecanum_controller/cmd_vel   # what twist_mux sends to the robot
ros2 topic echo /clock --once                 # Gazebo is running (teleop uses simulation time)
```

The session must be X11, or Wayland with XWayland (`echo $DISPLAY` not empty).

### Linux with an NVIDIA GPU

Intel and AMD GPUs work with `gui` (Mesa, through `/dev/dri`). With an NVIDIA GPU and its proprietary
driver, `gui` falls back to slow CPU rendering (`glxinfo -B` says `llvmpipe`); use `gui-nvidia`
instead. It needs the NVIDIA driver (`nvidia-smi` works on the host) and, once, the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

```bash
# after adding NVIDIA's apt repository (link above)
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
# then, as for gui
source docker/env.sh
docker compose -f docker/compose.yaml run --rm gui-nvidia
glxinfo -B                     # inside: "OpenGL vendor string: NVIDIA Corporation"
```

On a laptop with two GPUs (Intel/AMD + NVIDIA), `gui` uses the integrated GPU and `gui-nvidia` the
NVIDIA one (PRIME render offload).

## Windows (WSL2 + WSLg)

Once, in PowerShell: `wsl --install -d Ubuntu-24.04` (or `wsl --update` if WSL is already there),
then restart. Everything else happens in the **Ubuntu** terminal:

```bash
# Docker Engine inside Ubuntu (skip if you use Docker Desktop with WSL integration)
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER            # then close and reopen the Ubuntu terminal
# the repository, inside Ubuntu
git clone https://github.com/ddrozo31/caging_multirobot_ros2.git && cd caging_multirobot_ros2
source docker/env.sh                     # prints "(WSL2)" and the WSLg folder
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm test
docker compose -f docker/compose.yaml run --rm gui-wsl
```

Inside `gui-wsl`, `glxinfo -B` should say `OpenGL renderer string: D3D12 (<your GPU>)`. Then the demos
are the same as on Linux (three `gui-wsl` terminals for Demo 1). If the renderer is `llvmpipe` or
Gazebo does not open, use CPU rendering (slower, but it works):
`GALLIUM_DRIVER=llvmpipe docker compose -f docker/compose.yaml run --rm gui-wsl`
(`LIBGL_ALWAYS_SOFTWARE=1` alone does not work here: under WSLg it still draws with `d3d12`).
If Gazebo opens **black** and the log repeats `Requesting list of world names` (the Gazebo server did
not start; seen once, on the first launch after the build), stop with Ctrl+C and launch again.
Laptops with two GPUs: `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (or `Intel`, `AMD`) inside.

Joystick on Windows: USB devices do not reach WSL2 by default; use the keyboard, or attach the
joystick with `usbipd-win` (`usbipd attach --wsl --busid <id>`) and use the Linux `gui` service.

## Publishing for students (author)

Students do not build: they download the image from the GitHub container registry and use the kit
(`kit/STUDENT_GUIDE.md`). The repository and the image are public.

```bash
# once per computer: GitHub -> Settings -> Developer settings -> Personal access tokens (classic),
# scope write:packages; then
echo <token> | docker login ghcr.io -u ddrozo31 --password-stdin
bash docker/publish.sh          # checks src/ == v1.0 and a clean tree, builds, asks, pushes 3 tags
bash docker/make_kit.sh         # docker/dist/caging-docker-kit.zip -> course page, GitHub release
```

After the **first** push, make the package public once: GitHub → your profile → Packages →
`caging_multirobot_ros2` → Package settings → Change visibility → Public. Check from a machine that is not logged
in: `docker pull ghcr.io/ddrozo31/caging_multirobot_ros2:jazzy`.

ROS 2 discovery is limited to the computer (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, all services),
so laptops on the same classroom Wi-Fi do not see each other. To talk to a real robot or another
computer: `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET docker compose ...`.

# Caging simulator in Docker: student guide

Two mecanum robots with soft grippers cage a box and carry it along a trajectory (Tejada et al.,
2025, *J. Intelligent & Robotic Systems* 111:76), simulated in ROS 2 Jazzy and Gazebo Harmonic.
This kit runs the simulator in a container: you install only Docker, not ROS 2.

Lecture: Robotics, Mechatronics Engineering, Universidad EIA (David Rozo-Osorio).
Repository (code, step-by-step guides): <https://github.com/ddrozo31/caging_multirobot_ros2>.

## 1. What you need

| Your computer | Install |
|---|---|
| **Linux** (Ubuntu 22.04/24.04 or similar, x86-64) | Docker Engine (below) |
| **Windows 10 (21H2+) or 11** (x86-64) | WSL2 with Ubuntu 24.04, then Docker Engine inside Ubuntu (below) |
| macOS | not supported (no GPU access for the container, no display for Gazebo) |

**Processor: x86-64 (Intel or AMD) only.** The image is built for x86-64. ARM computers are not
supported yet: Apple Silicon Macs (M1–M4), Windows on ARM laptops (Snapdragon, Surface Pro 11) and
Raspberry Pi do not run it (`exec format error`, or too slow under emulation).

**Hardware.** The simulator computes the physics 1000 times per simulated second, so the speed of
the processor decides how fast it runs:

| | Minimum (the task finishes; the GUI is slow but usable) | Recommended |
|---|---|---|
| Processor | 4 cores / 8 threads, laptop **H series** or desktop, 2018 or newer (e.g. Core i5-8300H, Ryzen 5 4600H); **not** a low-power U series processor in a tablet or a fanless laptop | 6 cores or more (e.g. Core i7-9750H, Ryzen 7 5800H or newer) |
| Memory | 8 GB on Linux; **16 GB on Windows** (WSL2 gets half of it) | 16 GB |
| Graphics | any GPU with hardware OpenGL 3.3 or newer; the processor's integrated graphics are enough (Intel UHD 620 / Iris, AMD Radeon Vega or newer) | a recent integrated or a dedicated GPU |
| Disk | 20 GB free, SSD | 20 GB free, SSD |

For reference, a Surface Pro 7 (Core i5-1035G4, U series, 8 GB), below the minimum, still runs the
GUI demos (slowly) and finishes the four `experiments` runs, but each run takes up to 233 s of its
240 s limit: any background load makes it time out.

Keep a laptop **plugged in** and in its performance power mode. Native Linux runs faster than
Windows (WSL2). Also needed: an internet connection for the first download (about 3 GB).

**Windows only, once** (PowerShell as administrator), then restart:

```powershell
wsl --install -d Ubuntu-24.04
```

Open "Ubuntu 24.04" from the Start menu, create your user, and do everything below **in that
Ubuntu terminal**. (Docker Desktop with "WSL integration" for Ubuntu also works.)

**Docker Engine** (Linux, or Ubuntu inside WSL2), once:

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER      # then close the terminal and open a new one
docker run --rm hello-world        # must print "Hello from Docker!"
```

## 2. The kit

Get `caging-docker-kit.zip` from the course page or from the *Releases* page of the repository,
and unzip it (Windows: inside Ubuntu, e.g. copy it to your home folder with
`cp /mnt/c/Users/<you>/Downloads/caging-docker-kit.zip ~/`):

```bash
cd ~ && unzip caging-docker-kit.zip && cd caging-docker-kit
source env.sh              # in EVERY new terminal, inside this folder
docker compose pull        # download the simulator image, once (about 3 GB)
docker compose run --rm test
```

The last command must end with `Summary: 32 tests, 0 errors, 0 failures, 0 skipped`.

## 3. Open Gazebo and RViz

Pick the service for your computer:

| Computer | Service |
|---|---|
| Linux, Intel or AMD graphics | `gui` |
| Linux, NVIDIA graphics (driver + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)) | `gui-nvidia` |
| Windows (WSL2) | `gui-wsl` |

```bash
docker compose run --rm gui        # or gui-nvidia / gui-wsl; you get a shell "ros@...$"
glxinfo -B                         # check: your GPU's name, not "llvmpipe" (llvmpipe = slow, CPU)
```

Each terminal is its own container. Open more terminals the same way (`cd caging-docker-kit`,
`source env.sh`, `docker compose run --rm gui`); they all see the same ROS 2 topics. ROS 2 only looks
for other nodes on your own computer, so classmates on the same Wi-Fi do not drive your robot.

## 4. Demos of the lecture

**Demo 1: drive one robot** (three terminals):

```bash
ros2 launch caging_gazebo sim.launch.py          # terminal 1: Gazebo + RViz, wait for the robot
ros2 launch caging_bringup teleop.launch.py      # terminal 2: twist_mux (+ joystick)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key   # terminal 3
```

Keyboard (terminal 3 focused): `i` / `,` forward / back, `j` / `l` rotate, `J` / `L` (Shift) sideways,
`k` stop. Joystick (Linux): hold LB, left stick moves, right stick rotates.

**Demo 2: approach and closure** and **Demo 3: the whole task** (stop the previous demo with Ctrl+C):

```bash
ros2 launch caging_bringup caging.launch.py
ros2 launch caging_bringup task.launch.py auto_start:=true
```

**Table 2 of the paper without graphics** (about 8 minutes; results in `results/`):

```bash
docker compose run --rm experiments
```

Expected: `4 of 4 runs finished the lap`, e_dO about 0.10 m (waypoints) and 0.05 m (smooth).

## 5. When something fails

| Symptom | Fix |
|---|---|
| `permission denied ... docker.sock` | `sudo usermod -aG docker $USER`, then a new terminal |
| `cannot open display` / Gazebo does not open | `source env.sh` in this terminal; Linux: `echo $DISPLAY` must not be empty |
| `glxinfo -B` says `llvmpipe` | NVIDIA on Linux: use `gui-nvidia`; Windows: `wsl --update` in PowerShell; it still works, only slower |
| Gazebo black and the log repeats `Requesting list of world names` | the Gazebo server did not start: Ctrl+C and launch again |
| Windows: Gazebo window black or crashes | `GALLIUM_DRIVER=llvmpipe docker compose run --rm gui-wsl` (CPU rendering) |
| `amdgpu: os_same_file_description couldn't determine ...` | harmless warning of the AMD driver in a container; ignore it |
| The robot does not move with the keyboard | terminal 2 (`teleop.launch.py`) must be running; type in terminal 3 |
| `experiments`: runs end `timed out` (fewer than 4 of 4) | your computer runs the simulation too slowly for the 4-minute limit per run (see the hardware table in section 1): close other programs, plug in the laptop; the GUI demos still work, only slower |
| `results/`: permission denied | your user ID is not 1000 (`id -u`): `sudo chown -R $USER results` after the run |

## 6. Inside the container

The simulator's workspace is `/home/ros/ros2_ws` (packages `caging_*` in `src/`). Changes you make
inside a container are lost when you leave it (`--rm`); save your own files in `results/`, which is
the kit's `results/` folder on your computer.

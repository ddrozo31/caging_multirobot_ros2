# Step 2 (companion) — Inside the Mecanum Controller

*A guided tour of `caging_mecanum_controller` for readers who know some Python but little or no
C++.* Read [step_02_mecanum_controller.md](step_02_mecanum_controller.md) first: it tells
you **what** the controller does. This document explains **how** and **why**, line by line.

**Contents**

1. [Why ros2_control exists](#1-why-ros2_control-exists)
2. [The pieces of ros2_control](#2-the-pieces-of-ros2_control)
3. [From `ros2 launch` to a running controller](#3-from-ros2-launch-to-a-running-controller)
4. [C++ crash course (only what this controller uses)](#4-c-crash-course-only-what-this-controller-uses)
5. [The header file, line by line](#5-the-header-file-line-by-line)
6. [The source file, function by function](#6-the-source-file-function-by-function)
7. [Threads and "real-time": why the box and the real-time publisher](#7-threads-and-real-time)
8. [How a C++ class becomes a plugin](#8-how-a-c-class-becomes-a-plugin)
9. [Building: CMakeLists.txt and package.xml explained](#9-building-cmakeliststxt-and-packagexml-explained)
10. [The same controller in Python (pseudo-code)](#10-the-same-controller-in-python-pseudo-code)
11. [Inspecting a running controller](#11-inspecting-a-running-controller)
12. [Self-check questions](#12-self-check-questions)

---

## 1. Why ros2_control exists

Without ros2_control, a typical robot program looks like this:

```
read sensors  ->  compute  ->  send motor commands      (all in one node, written for ONE robot)
```

Problems:

- The code is tied to **one** piece of hardware. Moving from simulation to the real robot means
  rewriting it.
- Normal ROS topics have **unpredictable delays**. A motor loop that should run every 10 ms may
  sometimes wait 30 ms for a message.
- Every team re-implements the same things (reading joints, safety, switching controllers).

ros2_control splits the problem in two and puts a fixed-rate loop in the middle:

```
          HARDWARE side                                       CONTROLLER side
 (knows how to move THIS robot)                     (knows the math, works for ANY robot)

 ┌────────────────────────┐   state interfaces    ┌───────────────────────────┐
 │  Hardware interface    │ ───────────────────►  │  Controller               │
 │  (Gazebo / real board) │                       │  (our MecanumController)  │
 │                        │ ◄───────────────────  │                           │
 └────────────────────────┘  command interfaces   └───────────────────────────┘
                 ▲                                              ▲
                 └────────── controller_manager: every 10 ms ───┘
                               1. read()   hardware -> states
                               2. update() controllers: states -> commands
                               3. write()  commands -> hardware
```

**The key idea:** our controller never talks to Gazebo. It only reads and writes **numbers**
called *interfaces* (`wheel_0_joint/velocity`, …). To use the same controller on the real
ROSMASTER X3, only the hardware side changes (a different `<plugin>` in the URDF); the
controller code is untouched.

## 2. The pieces of ros2_control

| Piece | Analogy | Where it is in our project |
|-------|---------|----------------------------|
| **Joint** | a motor shaft | `wheel_0_joint` … `wheel_3_joint` (URDF) |
| **State interface** | a sensor value you may **read** | `wheel_i_joint/position`, `wheel_i_joint/velocity` |
| **Command interface** | a setpoint you may **write** | `wheel_i_joint/velocity` |
| **Hardware interface** (a.k.a. *system*) | the driver board | `gz_ros2_control/GazeboSimSystem` |
| **Resource manager** | the librarian that lends interfaces | inside the controller_manager |
| **Controller** | the brain of one task | `MecanumController`, `joint_state_broadcaster` |
| **Controller manager** | the conductor of the orchestra | started by the gz_ros2_control plugin |
| **Spawner** | the person who asks the conductor to start a musician | `spawner` nodes in `sim.launch.py` |

### Interfaces are *lent*, not shared

A command interface can be **claimed by only one controller at a time** (two controllers
writing the same motor would fight). The resource manager *lends* ("loans") the interfaces to
the controller when it is activated. That is why, in the code, the interfaces arrive in
vectors called `command_interfaces_` and `state_interfaces_` and their type is
`LoanedCommandInterface` / `LoanedStateInterface`.

State interfaces can be read by many controllers: `joint_state_broadcaster` and our controller
both read `wheel_i_joint/velocity`.

### Where each interface is declared

In [ros2_control.xacro](../../src/caging_description/urdf/ros2_control.xacro):

```xml
<joint name="wheel_0_joint">
    <command_interface name="velocity"/>   <!-- what a controller can WRITE -->
    <state_interface name="position"/>     <!-- what a controller can READ  -->
    <state_interface name="velocity"/>
</joint>
```

The full name of an interface is `<joint>/<interface>`, e.g. `wheel_0_joint/velocity`.

## 3. From `ros2 launch` to a running controller

What happens when you run `ros2 launch caging_gazebo sim.launch.py`:

```
1. Gazebo starts and loads the world.
2. "create" spawns the robot from /robot_description (our URDF).
3. Gazebo finds <plugin filename="gz_ros2_control-system"> in the URDF (gazebo.xacro)
   └─ it starts a controller_manager node INSIDE Gazebo
   └─ it reads the <ros2_control> tag and creates GazeboSimSystem for the 4 wheel joints
   └─ it loads config/ros2_controllers.yaml  (update_rate, controller types, parameters)
4. spawner joint_state_broadcaster   ─┐  each spawner asks the controller_manager to:
5. spawner mecanum_controller        ─┘  LOAD -> CONFIGURE -> ACTIVATE the controller
6. From now on, every 10 ms (update_rate: 100):
      read() -> update() of every ACTIVE controller -> write()
```

### The controller lifecycle

Controllers are *lifecycle* objects: they move through fixed states, and each transition calls
one of our functions.

```
  load ──► [unconfigured] ──configure──► [inactive] ──activate──► [active] ──┐
             on_init()                   on_configure()     on_activate()     │ update()
                                                                              │ every 10 ms
                                         [inactive] ◄──deactivate─────────────┘
                                                    on_deactivate()
```

Why so many steps? Because each one has a clear job, and errors are caught early:

| Function | Job | Allowed to be slow? |
|----------|-----|---------------------|
| `on_init` | declare parameters | yes |
| `on_configure` | read parameters, create topics, check configuration | yes |
| `on_activate` | get ready to move (reset state) | short |
| `update` | the control loop | **no — must finish in well under 10 ms** |
| `on_deactivate` | leave the robot safe (stop wheels) | short |

## 4. C++ crash course (only what this controller uses)

Each concept is shown next to its closest Python equivalent.

### 4.1 Header (`.hpp`) and source (`.cpp`) files

C++ usually splits a class in two files:

- the **header** `mecanum_controller.hpp` *declares* what exists (names, types) — like a table
  of contents;
- the **source** `mecanum_controller.cpp` *defines* how it works (the function bodies).

`#include "file.hpp"` copies the content of a header into the current file (≈ `import`).

```cpp
#ifndef CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_   // "include guard":
#define CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_   // prevents including the
...                                                          // same header twice
#endif
```

### 4.2 Types are written explicitly

Python figures out types at run time; C++ needs them when compiling:

| Python | C++ |
|--------|-----|
| `r = 0.0325` | `double r = 0.0325;` |
| `names = []` | `std::vector<std::string> names;` |
| `w = [0.0] * 4` | `std::array<double, 4> w{};` |
| `x = something()` | `auto x = something();` (`auto` = "compiler, deduce the type") |
| `None` / value | `std::optional<double>` (a box that may be empty) |

`std::` is the *namespace* of the C++ standard library (like `numpy.` in `numpy.array`).
`::` separates a namespace or class from a name: `controller_interface::CallbackReturn`.

Every statement ends with `;`, and blocks use `{ }` instead of indentation.

### 4.3 Classes and inheritance

```cpp
class MecanumController : public controller_interface::ControllerInterface
{
public:
  controller_interface::return_type update(...) override;
private:
  double wheel_radius_ = 0.0;
};
```

Python equivalent:

```python
class MecanumController(ControllerInterface):
    def update(self, time, period): ...
```

- `: public ControllerInterface` = "inherits from" (`class A(B)` in Python).
  `ControllerInterface` is the base class written by the ros2_control team; it already knows how
  to talk to the controller_manager. We only fill in the functions it asks for.
- `override` = "this function replaces one of the base class". The compiler checks that the
  base class really has a function with exactly this signature (a typo becomes an error, not a
  silent bug).
- `public:` functions can be called from outside (the controller_manager calls them);
  `private:` members are internal.
- Member variables end with `_` (`wheel_radius_`) by convention, so you can tell them apart
  from local variables. In Python you would write `self.wheel_radius`.
- `MyClass::function` defines a function that belongs to a class:
  `controller_interface::return_type MecanumController::update(...) { ... }`.

### 4.4 `.` versus `->`, and smart pointers

- `object.member` — use the object directly.
- `pointer->member` — follow a pointer to the object, then use the member.

A *pointer* is an address of an object. Raw pointers are error-prone, so modern C++ uses
**smart pointers** that delete the object automatically:

| Smart pointer | Meaning | In our code |
|---------------|---------|-------------|
| `std::shared_ptr<T>` | many owners; deleted when the last owner is gone | `rclcpp::Subscription<...>::SharedPtr` |
| `std::unique_ptr<T>` | exactly one owner | `rt_body_twist_pub_` |
| `std::make_unique<T>(args)` | creates a `unique_ptr` | in `on_configure` |

Python handles this automatically (every Python variable behaves like a `shared_ptr`).

### 4.5 References and `const`

```cpp
for (const auto & joint : wheel_joints_) { ... }
```

- `&` = *reference*: use the original object, do not copy it (cheap for strings).
- `const` = "I promise not to modify it". The compiler enforces the promise.
- A function marked `const` after its parentheses (e.g.
  `command_interface_configuration() const`) promises not to modify the object's members.

Python: `for joint in self.wheel_joints:` (Python never copies here).

### 4.6 Templates: the `<...>` after a name

`std::vector<std::string>`, `create_subscription<geometry_msgs::msg::Twist>(...)`,
`declare_parameter<double>(...)`: the type inside `<>` tells a generic class or function which
type to work with. Think of it as a type argument.

### 4.7 Lambdas (anonymous functions)

```cpp
[this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_box_.set(*msg); }
```

Python: `lambda msg: self.cmd_box.set(msg)`.

- `[this]` = the lambda may use the members of the current object (like capturing `self`).
- `*msg` = "the object the pointer points to" (*dereference*).

### 4.8 Enums as return values

`controller_interface::CallbackReturn::SUCCESS` / `ERROR` and
`controller_interface::return_type::OK` are named constants (like `Enum` in Python). The
controller_manager reads them to decide what to do next — e.g. if `on_configure` returns
`ERROR`, the controller is never activated.

### 4.9 `[[nodiscard]]` and `(void)`

`set_value()` returns `true`/`false` (did the write succeed?) and is marked `[[nodiscard]]`:
the compiler **warns** if you ignore the result. We either check it (`if (!...set_value(...))`)
or say explicitly "I know, I ignore it" with `(void)`.

## 5. The header file, line by line

File: [mecanum_controller.hpp](../../src/caging_mecanum_controller/include/caging_mecanum_controller/mecanum_controller.hpp)

```cpp
#include "controller_interface/controller_interface.hpp"   // base class
#include "geometry_msgs/msg/twist.hpp"                     // Twist message
#include "rclcpp/rclcpp.hpp"                               // ROS 2 C++ client library
#include "realtime_tools/realtime_publisher.hpp"           // see section 7
#include "realtime_tools/realtime_thread_safe_box.hpp"     // see section 7

namespace caging_mecanum_controller        // groups our names, avoids clashes with
{                                           // other packages (like a Python package name)

class MecanumController : public controller_interface::ControllerInterface
{
public:
  MecanumController() = default;           // constructor: nothing special to do
```

**The functions the base class requires** (the "contract" of every controller):

```cpp
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
```

Reading one of them: `on_configure` *returns* a `CallbackReturn`, is *called with* the
previous lifecycle state (we do not use it), and *overrides* the base-class version.

**The data the object keeps between calls** (Python: attributes set in `__init__`):

```cpp
private:
  std::vector<std::string> wheel_joints_;  // 4 joint names (parameter)
  double wheel_radius_ = 0.0;              // r_w            (parameter)
  double lx_ = 0.0;                        // l_x            (parameter)
  double ly_ = 0.0;                        // l_y            (parameter)

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;      // ~/cmd_vel
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> cmd_box_; // shared command
  geometry_msgs::msg::Twist last_cmd_;                                       // used by update()

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_pub_;   // ~/body_twist
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_body_twist_pub_;
};
```

Why keep `cmd_sub_` as a member even if we never use it again? Because a subscription only
lives as long as someone holds its pointer. If it were a local variable in `on_configure`, it
would be destroyed at the end of that function and no message would ever arrive.

## 6. The source file, function by function

File: [mecanum_controller.cpp](../../src/caging_mecanum_controller/src/mecanum_controller.cpp)

### 6.1 `on_init` — declare the parameters

```cpp
controller_interface::CallbackReturn MecanumController::on_init()
{
  auto node = get_node();
  node->declare_parameter<std::vector<std::string>>("wheel_joints", std::vector<std::string>{});
  node->declare_parameter<double>("wheel_radius", 0.0325);  // r_w [m]
  node->declare_parameter<double>("lx", 0.080);             // l_x [m]
  node->declare_parameter<double>("ly", 0.0845);            // l_y [m]
  return controller_interface::CallbackReturn::SUCCESS;
}
```

- Every controller owns a ROS 2 node (`get_node()`), named like the controller:
  `mecanum_controller`. Parameters, topics and logs live on that node.
- ROS 2 only accepts parameters that were **declared** first, with a type and a default.
- The values in `ros2_controllers.yaml` under `mecanum_controller: ros__parameters:` then
  override these defaults.

Python equivalent: `self.declare_parameter('wheel_radius', 0.0325)`.

> **Since Step 12** `wheel_radius`, `lx` and `ly` are no longer parameters: `on_configure` reads
> them from the robot description (URDF), the only place where they are written. Only
> `wheel_joints` (and the later Step 3–5 parameters) remain. See
> [step_12_single_source.md](step_12_single_source.md).

### 6.2 Interface configuration — what we want to borrow

```cpp
controller_interface::InterfaceConfiguration
MecanumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : wheel_joints_) {
    config.names.push_back(joint + "/velocity");  // e.g. "wheel_0_joint/velocity"
  }
  return config;
}
```

- `INDIVIDUAL` = "I list exactly the interfaces I want" (other options: `ALL`, `NONE`).
- `push_back` = Python `append`.
- **Order matters:** the resource manager gives us the interfaces in the same order we list
  them. Because `wheel_joints` is ordered 0, 1, 2, 3 (Fig. 6), `command_interfaces_[i]` is
  wheel *i* of Eq. 1. Exercise 2 of Step 2 shows what happens if the order is wrong.
- `state_interface_configuration()` is identical: we read the `velocity` state of each wheel.

These functions are called **after** `on_configure` (they need `wheel_joints_`) and **before**
`on_activate` (the interfaces must be lent before the loop starts).

### 6.3 `on_configure` — read parameters, create topics

```cpp
  wheel_joints_ = node->get_parameter("wheel_joints").as_string_array();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  ...
  if (wheel_joints_.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "'wheel_joints' must have exactly 4 names (wheel 0, 1, 2, 3).");
    return controller_interface::CallbackReturn::ERROR;
  }
```

**Fail early:** a wrong configuration stops the controller *before* it can move the robot.

```cpp
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_box_.set(*msg); });
```

- `"~/cmd_vel"`: the `~` means "inside my node's name", so the topic becomes
  `/mecanum_controller/cmd_vel`.
- The callback does **almost nothing**: it stores the message in the box. All the math happens
  in `update()` at a fixed rate. (Why? See section 7.)

```cpp
  body_twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("~/body_twist", rclcpp::SystemDefaultsQoS());
  rt_body_twist_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(body_twist_pub_);
```

A normal publisher wrapped in a real-time publisher (section 7). Topic:
`/mecanum_controller/body_twist`.

### 6.4 `on_activate` and `on_deactivate` — start and stop safely

```cpp
  last_cmd_ = geometry_msgs::msg::Twist();  // all zeros
  cmd_box_.set(last_cmd_);
```

When the controller (re)starts, an old command could still be stored. We reset it to zero, so
the robot never jumps at activation.

```cpp
  for (auto & wheel : command_interfaces_) {
    (void)wheel.set_value(0.0);  // stop all wheels
  }
```

When the controller is stopped, the hardware would otherwise keep the last velocity command.
We leave the wheels at zero.

### 6.5 `update` — the control loop

Called every 10 ms. Five parts (Step 3 adds a sixth one, the wheel odometry — see
[step_03_pose_feedback.md](step_03_pose_feedback.md)):

**1) Take the newest command**

```cpp
  const auto new_cmd = cmd_box_.try_get();
  if (new_cmd.has_value()) {
    last_cmd_ = new_cmd.value();
  }
```

`try_get()` returns an `std::optional`: a value if the box was free, empty if the callback
thread was writing at that exact moment. In that rare case we simply reuse `last_cmd_`.
`update()` must **never wait**.

**2) Inverse kinematics** (body twist → wheel speeds)

```cpp
  const std::array<double, 4> wheel_cmd = {
    (vx - vy - L * wz) / r,   // wheel 0, front-left
    (vx + vy + L * wz) / r,   // wheel 1, front-right
    (vx + vy - L * wz) / r,   // wheel 2, rear-left
    (vx - vy + L * wz) / r,   // wheel 3, rear-right
  };
  for (size_t i = 0; i < 4; ++i) {
    if (!command_interfaces_[i].set_value(wheel_cmd[i])) { ... warn ... }
  }
```

`size_t` is an unsigned integer type used for sizes and indices; `++i` means `i += 1`.
Writing to `command_interfaces_[i]` does **not** move the wheel immediately: the
controller_manager calls the hardware's `write()` after all controllers have updated.

**3) Read the measured wheel speeds**

```cpp
  std::array<double, 4> w{};
  for (size_t i = 0; i < 4; ++i) {
    const auto value = state_interfaces_[i].get_optional();
    if (!value.has_value()) {
      return controller_interface::return_type::OK;  // no data this cycle: try again next cycle
    }
    w[i] = value.value();
  }
```

These values were filled by the hardware's `read()` at the start of this cycle.

**4) Forward kinematics** (Eq. 1)

```cpp
  body_twist.linear.x  = r / 4.0 * ( w[0] + w[1] + w[2] + w[3]);
  body_twist.linear.y  = r / 4.0 * (-w[0] + w[1] + w[2] - w[3]);
  body_twist.angular.z = r / (4.0 * L) * (-w[0] + w[1] - w[2] + w[3]);
```

Note `4.0`, not `4`: in C++, `1 / 4` is integer division and equals `0`. Writing `4.0` keeps
the arithmetic in floating point.

**5) Publish without blocking**

```cpp
  rt_body_twist_pub_->try_publish(body_twist);
```

### 6.6 The last line: the plugin export

```cpp
PLUGINLIB_EXPORT_CLASS(
  caging_mecanum_controller::MecanumController, controller_interface::ControllerInterface)
```

A *macro* (code generated at compile time) that registers our class so it can be created by
name at run time. See section 8.

## 7. Threads and "real-time"

### Two threads touch the command

```
 Thread A (ROS executor)                    Thread B (controller_manager loop, 100 Hz)
 ──────────────────────                     ──────────────────────────────────────────
 message arrives on ~/cmd_vel               update():
   └─ callback: cmd_box_.set(msg)  ───────►   cmd_box_.try_get() → last_cmd_
```

If both threads used a plain variable at the same moment, `update()` could read a Twist that is
half old and half new (e.g. the new `linear.x` with the old `angular.z`). This is a
**data race**.

`RealtimeThreadSafeBox` protects the value with a lock (*mutex*):

- `set()` (callback side) may wait for the lock — the callback is allowed to wait;
- `try_get()` (update side) **never waits**: if the lock is busy, it returns empty and we keep
  the previous command for one more cycle (10 ms).

### What "real-time" means here

Real-time does not mean "fast"; it means **predictable**. `update()` must finish before the
next cycle, every cycle. Therefore inside `update()` we avoid anything that can take an
unknown amount of time:

| Avoid in `update()` | Why | What we do instead |
|---------------------|-----|--------------------|
| waiting for a lock | the other thread may hold it for long | `try_get()` |
| publishing directly | may block inside the network stack | `RealtimePublisher::try_publish()` copies the message, a background thread sends it |
| allocating memory (`new`, growing vectors) | the allocator can be slow | fixed-size `std::array<double, 4>` |
| printing every cycle | console I/O is slow | `RCLCPP_WARN_THROTTLE` (at most once per second) |

In simulation nothing breaks if we are careless, but on the real robot these rules are what
keep the motors from jerking.

## 8. How a C++ class becomes a plugin

The controller_manager is a program that already exists. How can it run **our** class, which
was written after it was compiled? With **plugins**:

```
 1. We compile our class into a shared library:  libcaging_mecanum_controller.so
 2. mecanum_controller_plugin.xml says:
        name "caging_mecanum_controller/MecanumController"
        is class caging_mecanum_controller::MecanumController
        inside library caging_mecanum_controller
        and it is a controller_interface::ControllerInterface
 3. PLUGINLIB_EXPORT_CLASS(...) in the .cpp registers the class inside the library.
 4. ros2_controllers.yaml says:   mecanum_controller:
                                    type: caging_mecanum_controller/MecanumController
 5. At run time, pluginlib finds the XML, opens the .so file and creates the object.
```

A *shared library* (`.so` on Linux, `.dll` on Windows) is compiled code without a `main()`
that other programs load at run time — similar to a Python module that another program
imports. That is why our controller is not a node with `main()`: the controller_manager owns
the loop and **loads** us into it (exercise 4 of Step 2).

## 9. Building: CMakeLists.txt and package.xml explained

`colcon build` → for an `ament_cmake` package runs **CMake**, which runs the C++ compiler.
[CMakeLists.txt](../../src/caging_mecanum_controller/CMakeLists.txt) is the recipe:

| Lines | Meaning |
|-------|---------|
| `find_package(controller_interface REQUIRED)` … | find the headers and libraries of each dependency |
| `add_library(${PROJECT_NAME} SHARED src/mecanum_controller.cpp)` | compile our `.cpp` into `libcaging_mecanum_controller.so` |
| `target_compile_features(... cxx_std_17)` | use the C++17 standard (needed for `std::optional`) |
| `target_include_directories(...)` | where to find our own header (`include/`) |
| `ament_target_dependencies(...)` | link our library with the dependencies |
| `pluginlib_export_plugin_description_file(controller_interface mecanum_controller_plugin.xml)` | install the plugin XML and register it for the `controller_interface` base class |
| `install(TARGETS ...)`, `install(DIRECTORY include/ ...)` | copy the `.so` and headers into `install/` |
| `ament_export_*`, `ament_package()` | let other packages find ours |

[package.xml](../../src/caging_mecanum_controller/package.xml) lists the same dependencies with
`<depend>` (needed to build **and** run). `rosdep` uses it to install missing packages, and
colcon uses it to build packages in the right order.

After `colcon build`, look for the result:

```bash
ls install/caging_mecanum_controller/lib/          # libcaging_mecanum_controller.so
ls install/caging_mecanum_controller/share/caging_mecanum_controller/   # plugin XML
```

## 10. The same controller in Python (pseudo-code)

ros2_control controllers cannot be written in Python, but this pseudo-code shows the logic
without the C++ syntax:

```python
class MecanumController(ControllerInterface):

    def on_init(self):
        self.declare_parameter('wheel_joints', [])
        self.declare_parameter('wheel_radius', 0.0325)   # Step 12: read from the URDF instead
        self.declare_parameter('lx', 0.080)
        self.declare_parameter('ly', 0.0845)

    def command_interface_configuration(self):
        return [f'{j}/velocity' for j in self.wheel_joints]   # order = wheel index i

    def state_interface_configuration(self):
        return [f'{j}/velocity' for j in self.wheel_joints]

    def on_configure(self):
        self.wheel_joints = self.get_parameter('wheel_joints')
        self.r, self.lx, self.ly = ...
        self.cmd_box = ThreadSafeBox(Twist())
        self.create_subscription(Twist, '~/cmd_vel', lambda msg: self.cmd_box.set(msg))
        self.pub = RealtimePublisher(self.create_publisher(Twist, '~/body_twist'))

    def on_activate(self):
        self.last_cmd = Twist()                    # start at rest

    def update(self, time, period):                # every 10 ms
        cmd = self.cmd_box.try_get()
        if cmd is not None:
            self.last_cmd = cmd
        vx, vy, wz = self.last_cmd.linear.x, self.last_cmd.linear.y, self.last_cmd.angular.z
        r, L = self.r, self.lx + self.ly

        wheel_cmd = [(vx - vy - L*wz)/r, (vx + vy + L*wz)/r,    # inverse Eq. 1
                     (vx + vy - L*wz)/r, (vx - vy + L*wz)/r]
        for i in range(4):
            self.command_interfaces[i].set_value(wheel_cmd[i])

        w = [self.state_interfaces[i].get_value() for i in range(4)]
        twist = Twist()                                          # forward Eq. 1
        twist.linear.x  = r/4 * ( w[0] + w[1] + w[2] + w[3])
        twist.linear.y  = r/4 * (-w[0] + w[1] + w[2] - w[3])
        twist.angular.z = r/(4*L) * (-w[0] + w[1] - w[2] + w[3])
        self.pub.try_publish(twist)

    def on_deactivate(self):
        for ci in self.command_interfaces:
            ci.set_value(0.0)                      # stop the wheels
```

Compare it with the `.cpp` file: the structure is the same; C++ adds types, `;`, `{ }` and the
real-time tools.

## 11. Inspecting a running controller

With the simulation running:

```bash
ros2 control list_controllers              # name, type and state of each controller
ros2 control list_hardware_interfaces      # every interface, and which ones are claimed
ros2 control list_controller_types         # all controller plugins pluginlib can find
ros2 param list /mecanum_controller        # parameters of our controller
ros2 param get /mecanum_controller wheel_joints   # (wheel_radius was a parameter until Step 11)

ros2 control set_controller_state mecanum_controller inactive   # calls on_deactivate()
ros2 control set_controller_state mecanum_controller active     # calls on_activate()

ros2 topic echo /mecanum_controller/body_twist
ros2 topic hz /joint_states                # ~100 Hz = update_rate
```

In `list_hardware_interfaces`, the four `wheel_i_joint/velocity` command interfaces appear as
`[claimed]` while `mecanum_controller` is active, and `[unclaimed]` after you deactivate it.

## 12. Self-check questions

1. Which three steps does the controller_manager run every cycle, and in which order?
2. Why can two controllers read `wheel_0_joint/velocity` but only one can write it?
3. What would happen if `cmd_sub_` were a local variable inside `on_configure`?
4. Why does the `~/cmd_vel` callback only store the message instead of computing the wheel
   speeds?
5. What is the difference between `cmd_box_.set()` and `cmd_box_.try_get()`, and why does each
   side use the one it uses?
6. What is the value of `1 / 4` in C++? And of `1 / 4.0`?
7. List the files that must agree on the name `caging_mecanum_controller/MecanumController`.
8. Deactivate the controller with `ros2 control set_controller_state`. Which function of our
   class runs, and what happens to the robot?

// Mecanum controller for ros2_control.
//
// Implements Eq. (1) of Tejada et al. (2025) in BOTH directions:
//
//   INVERSE (command):  body twist [vx, vy, wz]  ->  4 wheel speeds   -> wheel joints
//   FORWARD (feedback): 4 measured wheel speeds  ->  body twist       -> topic ~/body_twist
//
// In the simulation, ~/body_twist is sent to Gazebo (VelocityControl system),
// which moves the chassis. See docs/steps/step_02_mecanum_controller.md.

#ifndef CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace caging_mecanum_controller
{

class MecanumController : public controller_interface::ControllerInterface
{
public:
  MecanumController() = default;

  // ---- Functions that EVERY ros2_control controller must implement ----

  // Called once, when the controller is loaded: declare the parameters.
  controller_interface::CallbackReturn on_init() override;

  // Which joint interfaces we WRITE (commands) and READ (states).
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Lifecycle: configure -> activate -> (running) -> deactivate
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // The control loop: called by the controller_manager at "update_rate" Hz.
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- Parameters (from the controllers YAML file) ----
  std::vector<std::string> wheel_joints_;  // 4 joint names, order i = 0, 1, 2, 3 (Fig. 6)
  double wheel_radius_ = 0.0;              // r_w
  double lx_ = 0.0;                        // l_x
  double ly_ = 0.0;                        // l_y

  // ---- Command input: topic ~/cmd_vel ----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  // The subscriber callback and update() run in DIFFERENT threads.
  // This "box" lets them share the last command safely.
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> cmd_box_;
  geometry_msgs::msg::Twist last_cmd_;     // last command used by update()

  // ---- Feedback output: topic ~/body_twist ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_pub_;
  // Real-time safe wrapper: update() never waits for the network.
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_body_twist_pub_;
};

}  // namespace caging_mecanum_controller

#endif  // CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

// Mecanum controller for ros2_control — implementation.
// Read the header (mecanum_controller.hpp) first.

#include "caging_mecanum_controller/mecanum_controller.hpp"

#include <array>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace caging_mecanum_controller
{

// ============================================================================
// on_init: declare the parameters (name and default value).
// Real values come from the controllers YAML file.
// ============================================================================
controller_interface::CallbackReturn MecanumController::on_init()
{
  auto node = get_node();
  node->declare_parameter<std::vector<std::string>>("wheel_joints", std::vector<std::string>{});
  node->declare_parameter<double>("wheel_radius", 0.0325);  // r_w [m]
  node->declare_parameter<double>("lx", 0.080);             // l_x [m]
  node->declare_parameter<double>("ly", 0.0845);            // l_y [m]
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interfaces: we WRITE the velocity of each wheel joint ...
// ============================================================================
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

// ... and we READ the (measured) velocity of each wheel joint.
controller_interface::InterfaceConfiguration
MecanumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : wheel_joints_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

// ============================================================================
// on_configure: read parameters, create the subscriber and the publisher.
// ============================================================================
controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // --- Read parameters ---
  wheel_joints_ = node->get_parameter("wheel_joints").as_string_array();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  lx_ = node->get_parameter("lx").as_double();
  ly_ = node->get_parameter("ly").as_double();

  if (wheel_joints_.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "'wheel_joints' must have exactly 4 names (wheel 0, 1, 2, 3).");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node->get_logger(), "r_w = %.4f m, l_x = %.4f m, l_y = %.4f m", wheel_radius_, lx_, ly_);

  // --- Subscriber: desired body twist on ~/cmd_vel  (=> /mecanum_controller/cmd_vel) ---
  // The callback only stores the message; the math happens in update().
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_box_.set(*msg); });

  // --- Publisher: measured body twist on ~/body_twist  (=> /mecanum_controller/body_twist) ---
  body_twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("~/body_twist", rclcpp::SystemDefaultsQoS());
  rt_body_twist_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(body_twist_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate / on_deactivate: start and stop with the robot at rest.
// ============================================================================
controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
  last_cmd_ = geometry_msgs::msg::Twist();  // all zeros
  cmd_box_.set(last_cmd_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & wheel : command_interfaces_) {
    (void)wheel.set_value(0.0);  // stop all wheels
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// update: the control loop (runs at update_rate Hz).
// ============================================================================
controller_interface::return_type MecanumController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double r = wheel_radius_;
  const double L = lx_ + ly_;  // (l_x + l_y) appears in the third row of Eq. 1

  // ---------------------------------------------------------------
  // 1) Get the newest command (if the box is busy, keep the last one)
  // ---------------------------------------------------------------
  const auto new_cmd = cmd_box_.try_get();
  if (new_cmd.has_value()) {
    last_cmd_ = new_cmd.value();
  }
  const double vx = last_cmd_.linear.x;   // x_dot_R  : forward velocity  [m/s]
  const double vy = last_cmd_.linear.y;   // y_dot_R  : lateral velocity  [m/s]
  const double wz = last_cmd_.angular.z;  // theta_dot: angular velocity  [rad/s]

  // ---------------------------------------------------------------
  // 2) INVERSE kinematics (inverse of Eq. 1): body twist -> wheel speeds [rad/s]
  //
  //    theta_dot_0 = (vx - vy - L*wz) / r     front-left
  //    theta_dot_1 = (vx + vy + L*wz) / r     front-right
  //    theta_dot_2 = (vx + vy - L*wz) / r     rear-left
  //    theta_dot_3 = (vx - vy + L*wz) / r     rear-right
  // ---------------------------------------------------------------
  const std::array<double, 4> wheel_cmd = {
    (vx - vy - L * wz) / r,
    (vx + vy + L * wz) / r,
    (vx + vy - L * wz) / r,
    (vx - vy + L * wz) / r,
  };

  for (size_t i = 0; i < 4; ++i) {
    if (!command_interfaces_[i].set_value(wheel_cmd[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Could not write the command of wheel %zu", i);
    }
  }

  // ---------------------------------------------------------------
  // 3) Read the MEASURED wheel speeds [rad/s]
  // ---------------------------------------------------------------
  std::array<double, 4> w{};
  for (size_t i = 0; i < 4; ++i) {
    const auto value = state_interfaces_[i].get_optional();
    if (!value.has_value()) {
      return controller_interface::return_type::OK;  // no data this cycle: try again next cycle
    }
    w[i] = value.value();
  }

  // ---------------------------------------------------------------
  // 4) FORWARD kinematics (Eq. 1): wheel speeds -> body twist
  //
  //    x_dot_R   = r/4       * (  w0 + w1 + w2 + w3)
  //    y_dot_R   = r/4       * (- w0 + w1 + w2 - w3)
  //    theta_dot = r/(4 L)   * (- w0 + w1 - w2 + w3)
  // ---------------------------------------------------------------
  geometry_msgs::msg::Twist body_twist;
  body_twist.linear.x = r / 4.0 * (w[0] + w[1] + w[2] + w[3]);
  body_twist.linear.y = r / 4.0 * (-w[0] + w[1] + w[2] - w[3]);
  body_twist.angular.z = r / (4.0 * L) * (-w[0] + w[1] - w[2] + w[3]);

  // ---------------------------------------------------------------
  // 5) Publish the body twist (Gazebo uses it to move the chassis).
  //    try_publish never blocks: if the publisher is busy, we skip one cycle.
  // ---------------------------------------------------------------
  rt_body_twist_pub_->try_publish(body_twist);

  return controller_interface::return_type::OK;
}

}  // namespace caging_mecanum_controller

// Make the class loadable by pluginlib (must match mecanum_controller_plugin.xml)
PLUGINLIB_EXPORT_CLASS(
  caging_mecanum_controller::MecanumController, controller_interface::ControllerInterface)

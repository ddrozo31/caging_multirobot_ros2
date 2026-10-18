// Mecanum controller for ros2_control.
//
// Implements Eq. (1) of Tejada et al. (2025) in BOTH directions:
//
//   INVERSE (command):  body twist [vx, vy, wz]  ->  4 wheel speeds   -> wheel joints
//   FORWARD (feedback): 4 measured wheel speeds  ->  body twist       -> topic ~/body_twist
//
// In the simulation, ~/body_twist is sent to Gazebo (VelocityControl system),
// which moves the chassis. See docs/steps/step_02_mecanum_controller.md.
//
// It also computes WHEEL ODOMETRY (Step 3): it integrates the forward-kinematics
// twist over time (Eq. 3 + Eq. 5) and publishes ~/odom and the TF odom -> base_link.
// Optional errors (noise, wrong wheel radius) can be added to the odometry ONLY,
// to see how it drifts away from the ground truth. See docs/steps/step_03_pose_feedback.md.

#ifndef CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
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
  // Each received command gets a number (1, 2, 3, ...), so update() can tell a NEW
  // command from the old one still stored in the box (needed for the timeout).
  struct NumberedCommand
  {
    geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
    uint64_t number = 0;
  };
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  uint64_t commands_received_ = 0;         // counter, used only by the subscriber callback
  // The subscriber callback and update() run in DIFFERENT threads.
  // This "box" lets them share the last command safely.
  realtime_tools::RealtimeThreadSafeBox<NumberedCommand> cmd_box_;
  geometry_msgs::msg::Twist last_cmd_;     // last command used by update()

  // ---- Command timeout (Step 5) ----
  // If no new command arrives for cmd_timeout_ seconds, the robot stops.
  double cmd_timeout_ = 0.5;               // [s]
  uint64_t last_cmd_number_ = 0;           // number of the last command used
  rclcpp::Time last_cmd_time_;             // when update() got it (simulation clock)
  bool has_cmd_time_ = false;              // false until the first command arrives

  // ---- Feedback output: topic ~/body_twist ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_pub_;
  // Real-time safe wrapper: update() never waits for the network.
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_body_twist_pub_;

  // ---- Wheel odometry (Step 3) ----
  // Parameters
  std::string odom_frame_;           // parent frame of the odometry ("odom")
  std::string base_frame_;           // robot frame ("base_link")
  bool publish_tf_ = true;           // publish the TF odom -> base_link?
  double odom_noise_stddev_ = 0.0;   // random error added to each wheel speed [rad/s]
  double odom_radius_scale_ = 1.0;   // systematic error: radius used by the odometry = scale * r_w

  // Estimated pose of the robot in the odom frame
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_theta_ = 0.0;

  // Random number generator for the noise (normal / Gaussian distribution)
  std::mt19937 random_generator_;
  std::normal_distribution<double> noise_;

  // Publishers: ~/odom (nav_msgs/Odometry) and /tf (odom -> base_link)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_pub_;
};

}  // namespace caging_mecanum_controller

#endif  // CAGING_MECANUM_CONTROLLER__MECANUM_CONTROLLER_HPP_

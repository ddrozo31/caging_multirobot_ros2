// Mecanum controller for ros2_control — implementation.
// Read the header (mecanum_controller.hpp) first.

#include "caging_mecanum_controller/mecanum_controller.hpp"

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

namespace caging_mecanum_controller
{

// ============================================================================
// on_init: declare the parameters (name and default value).
// Real values come from the controllers YAML file.
// r_w, l_x and l_y are NOT parameters: they are read from the URDF (Step 12).
// ============================================================================
controller_interface::CallbackReturn MecanumController::on_init()
{
  auto node = get_node();
  node->declare_parameter<std::vector<std::string>>("wheel_joints", std::vector<std::string>{});
  // Safety (Step 5): stop the robot if no command arrives for this time [s]
  node->declare_parameter<double>("cmd_timeout", 0.5);

  // Wheel odometry (Step 3)
  node->declare_parameter<std::string>("odom_frame", "odom");
  node->declare_parameter<std::string>("base_frame", "base_link");
  node->declare_parameter<bool>("publish_tf", true);
  // Several robots (Step 5): add the namespace to the frame names (leader/odom, leader/base_link)
  node->declare_parameter<bool>("tf_frame_prefix_enable", true);
  node->declare_parameter<double>("odom_noise_stddev", 0.0);   // [rad/s], 0 = no noise
  node->declare_parameter<double>("odom_radius_scale", 1.0);   // 1 = no systematic error
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

  if (wheel_joints_.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "'wheel_joints' must have exactly 4 names (wheel 0, 1, 2, 3).");
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- Robot dimensions (Step 12): from the robot description, the ONLY place they are written ---
  if (!read_dimensions_from_urdf()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    node->get_logger(), "From the URDF: r_w = %.4f m, l_x = %.4f m, l_y = %.4f m",
    wheel_radius_, lx_, ly_);

  // --- Subscriber: desired body twist on ~/cmd_vel  (=> /mecanum_controller/cmd_vel) ---
  // The callback only stores the message; the math happens in update().
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      commands_received_++;
      cmd_box_.set(NumberedCommand{*msg, commands_received_});
    });
  cmd_timeout_ = node->get_parameter("cmd_timeout").as_double();
  RCLCPP_INFO(node->get_logger(), "Command timeout = %.2f s", cmd_timeout_);

  // --- Publisher: measured body twist on ~/body_twist  (=> /mecanum_controller/body_twist) ---
  body_twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("~/body_twist", rclcpp::SystemDefaultsQoS());
  rt_body_twist_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(body_twist_pub_);

  // --- Wheel odometry (Step 3) ---
  odom_frame_ = node->get_parameter("odom_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();

  // Frame prefix (Step 5). get_namespace() is "/" for one robot and "/leader" for the leader.
  // Python equivalent:
  //   ns = node.get_namespace().strip('/')           # "" or "leader"
  //   if ns: odom_frame = ns + '/' + odom_frame      # "leader/odom"
  // This is the same convention as robot_state_publisher's frame_prefix ("leader/").
  std::string ns = node->get_namespace();
  if (node->get_parameter("tf_frame_prefix_enable").as_bool() && ns != "/") {
    std::string prefix = ns.substr(1) + "/";   // "/leader" -> "leader/"
    odom_frame_ = prefix + odom_frame_;
    base_frame_ = prefix + base_frame_;
  }
  RCLCPP_INFO(node->get_logger(), "Odometry frames: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
  publish_tf_ = node->get_parameter("publish_tf").as_bool();
  odom_noise_stddev_ = node->get_parameter("odom_noise_stddev").as_double();
  odom_radius_scale_ = node->get_parameter("odom_radius_scale").as_double();
  RCLCPP_INFO(
    node->get_logger(), "Odometry errors: noise stddev = %.3f rad/s, radius scale = %.3f",
    odom_noise_stddev_, odom_radius_scale_);

  // Gaussian noise: mean 0, standard deviation odom_noise_stddev_ (fixed seed = repeatable runs)
  random_generator_ = std::mt19937(42);
  noise_ = std::normal_distribution<double>(0.0, odom_noise_stddev_);

  // ~/odom  (=> /mecanum_controller/odom)
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
  rt_odom_pub_ = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

  // /tf  (absolute name: every TF publisher in ROS 2 uses the same topic)
  tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
  rt_tf_pub_ = std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate / on_deactivate: start and stop with the robot at rest.
// ============================================================================
controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
  last_cmd_ = geometry_msgs::msg::Twist();  // all zeros
  has_cmd_time_ = false;                    // no command yet: the robot stays still

  // The odometry starts at the origin of the odom frame
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;
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
// read_dimensions_from_urdf (Step 12): r_w, l_x and l_y of Eq. 1 from the URDF.
//
// The robot description (robot_core.xacro -> URDF) is the ONLY place where the
// dimensions are written. The controller_manager gives every controller the same
// URDF that robot_state_publisher publishes; get_robot_description() returns it.
//
//   l_x, l_y : position of each wheel joint in base_link (wheel joints are children
//              of base_link in robot_core.xacro). Fig. 6 order and signs:
//                wheel 0 (front-left)  at (+l_x, +l_y)    wheel 1 (front-right) at (+l_x, -l_y)
//                wheel 2 (rear-left)   at (-l_x, +l_y)    wheel 3 (rear-right)  at (-l_x, -l_y)
//              All four wheels must give the SAME l_x and l_y: this also checks the
//              order of 'wheel_joints' in the YAML file.
//   r_w      : radius of the collision cylinder of the wheel link.
// ============================================================================
bool MecanumController::read_dimensions_from_urdf()
{
  auto logger = get_node()->get_logger();

  // 1) Parse the URDF text into a tree of links and joints
  urdf::Model model;
  if (get_robot_description().empty() || !model.initString(get_robot_description())) {
    RCLCPP_ERROR(logger, "Cannot read the robot description (URDF).");
    return false;
  }

  // 2) l_x and l_y from the position of each wheel joint
  const std::array<double, 4> sign_x = {+1.0, +1.0, -1.0, -1.0};
  const std::array<double, 4> sign_y = {+1.0, -1.0, +1.0, -1.0};
  for (size_t i = 0; i < 4; ++i) {
    auto joint = model.getJoint(wheel_joints_[i]);
    if (!joint) {
      RCLCPP_ERROR(logger, "Joint '%s' is not in the URDF.", wheel_joints_[i].c_str());
      return false;
    }
    const auto & position = joint->parent_to_joint_origin_transform.position;   // [m]
    const double lx_i = sign_x[i] * position.x;
    const double ly_i = sign_y[i] * position.y;
    if (i == 0) {
      // wheel 0 must be front-left: x > 0 and y > 0
      if (position.x <= 0.0 || position.y <= 0.0) {
        RCLCPP_ERROR(
          logger, "Wheel 0 ('%s') is at (%.4f, %.4f): wheel 0 must be front-left (x > 0, y > 0). "
          "Check the order of 'wheel_joints' (Fig. 6).", wheel_joints_[0].c_str(), position.x, position.y);
        return false;
      }
      lx_ = lx_i;   // wheel 0 gives the values ...
      ly_ = ly_i;
    } else if (std::abs(lx_i - lx_) > 1e-6 || std::abs(ly_i - ly_) > 1e-6) {
      // ... and the other three must agree (same distances, signs of Fig. 6)
      RCLCPP_ERROR(
        logger, "Wheel %zu ('%s') is at (%.4f, %.4f): it does not match Fig. 6 "
        "(wrong order in 'wheel_joints', or the wheels are not symmetric).",
        i, wheel_joints_[i].c_str(), position.x, position.y);
      return false;
    }
  }
  // 3) r_w from the collision cylinder of the wheel link
  auto wheel_link = model.getLink(model.getJoint(wheel_joints_[0])->child_link_name);
  if (!wheel_link || !wheel_link->collision ||
    wheel_link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    RCLCPP_ERROR(logger, "The wheel link needs a <collision> with a <cylinder>.");
    return false;
  }
  // The geometry is stored as a general "Geometry"; we know it is a cylinder: cast it.
  wheel_radius_ = std::static_pointer_cast<urdf::Cylinder>(wheel_link->collision->geometry)->radius;
  return true;
}

// ============================================================================
// update: the control loop (runs at update_rate Hz).
// ============================================================================
controller_interface::return_type MecanumController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double r = wheel_radius_;
  const double L = lx_ + ly_;  // (l_x + l_y) appears in the third row of Eq. 1

  // ---------------------------------------------------------------
  // 1) Get the newest command (if the box is busy, keep the last one)
  // ---------------------------------------------------------------
  const auto new_cmd = cmd_box_.try_get();
  if (new_cmd.has_value() && new_cmd->number != last_cmd_number_) {
    last_cmd_ = new_cmd->twist;          // a NEW command arrived
    last_cmd_number_ = new_cmd->number;
    last_cmd_time_ = time;
    has_cmd_time_ = true;
  }
  double vx = last_cmd_.linear.x;   // x_dot_R  : forward velocity  [m/s]
  double vy = last_cmd_.linear.y;   // y_dot_R  : lateral velocity  [m/s]
  double wz = last_cmd_.angular.z;  // theta_dot: angular velocity  [rad/s]

  // Timeout (Step 5): one late message is fine (the last command is kept), but if
  // commands stop for longer than cmd_timeout_ (crashed node, lost connection) the
  // robot must stop instead of driving on with an old command.
  //   Python equivalent:  if now - last_cmd_time > cmd_timeout: vx = vy = wz = 0
  if (!has_cmd_time_ || (time - last_cmd_time_).seconds() > cmd_timeout_) {
    if (has_cmd_time_ && (vx != 0.0 || vy != 0.0 || wz != 0.0)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "No command for more than %.2f s: stopping the robot", cmd_timeout_);
    }
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

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

  // ---------------------------------------------------------------
  // 6) WHEEL ODOMETRY (Step 3): what the robot "believes" from its wheels.
  //
  //    a) Wheel speeds as measured by imperfect encoders: add random noise.
  //    b) Forward Eq. 1 with the radius the odometry THINKS the wheel has
  //       (odom_radius_scale != 1 simulates worn tires or wheel slip).
  //    c) Rotate the twist to the odom frame (Eq. 3) and integrate it over
  //       one period with Euler (Eq. 5):  x(k) = x(k-1) + x_dot * Ts
  // ---------------------------------------------------------------
  std::array<double, 4> w_odom{};
  for (size_t i = 0; i < 4; ++i) {
    w_odom[i] = w[i] + (odom_noise_stddev_ > 0.0 ? noise_(random_generator_) : 0.0);   // a)
  }
  const double r_odom = r * odom_radius_scale_;                                         // b)
  const double vx_odom = r_odom / 4.0 * (w_odom[0] + w_odom[1] + w_odom[2] + w_odom[3]);
  const double vy_odom = r_odom / 4.0 * (-w_odom[0] + w_odom[1] + w_odom[2] - w_odom[3]);
  const double wz_odom = r_odom / (4.0 * L) * (-w_odom[0] + w_odom[1] - w_odom[2] + w_odom[3]);

  const double Ts = period.seconds();                                                    // c)
  const double c = std::cos(odom_theta_);
  const double s = std::sin(odom_theta_);
  odom_x_ += (c * vx_odom - s * vy_odom) * Ts;   // Eq. 3: robot frame {R} -> odom frame
  odom_y_ += (s * vx_odom + c * vy_odom) * Ts;
  odom_theta_ += wz_odom * Ts;
  odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));  // keep in [-pi, pi]

  // Orientation as a quaternion. For a rotation theta about z only:
  //   q = (x, y, z, w) = (0, 0, sin(theta/2), cos(theta/2))
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(odom_theta_ / 2.0);
  q.w = std::cos(odom_theta_ / 2.0);

  // ~/odom message: pose in the odom frame, twist in the robot frame
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_odom;
  odom.twist.twist.linear.y = vy_odom;
  odom.twist.twist.angular.z = wz_odom;
  rt_odom_pub_->try_publish(odom);

  // TF odom -> base_link (lets RViz place the robot model in the world)
  if (publish_tf_) {
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.resize(1);
    tf_msg.transforms[0].header = odom.header;
    tf_msg.transforms[0].child_frame_id = base_frame_;
    tf_msg.transforms[0].transform.translation.x = odom_x_;
    tf_msg.transforms[0].transform.translation.y = odom_y_;
    tf_msg.transforms[0].transform.rotation = q;
    rt_tf_pub_->try_publish(tf_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace caging_mecanum_controller

// Make the class loadable by pluginlib (must match mecanum_controller_plugin.xml)
PLUGINLIB_EXPORT_CLASS(
  caging_mecanum_controller::MecanumController, controller_interface::ControllerInterface)

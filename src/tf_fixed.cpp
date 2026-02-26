#include "tf_fixed/tf_fixed.hpp"

namespace tf_fixed
{
  
TfFixed::TfFixed()
: Node("tf_fixed"), tf_fixed_msg_(std::make_shared<geometry_msgs::msg::TransformStamped>())
{
  declare_parameter("period_ms", 100);
  declare_parameter("frame_id", "world");
  declare_parameter("child_frame_id", "map");
  declare_parameter("translation.x", 0.0);
  declare_parameter("translation.y", 0.0);
  declare_parameter("translation.z", 0.0);
  declare_parameter("rotation.quaternion.x", 0.0);
  declare_parameter("rotation.quaternion.y", 0.0);
  declare_parameter("rotation.quaternion.z", 0.0);
  declare_parameter("rotation.quaternion.w", 1.0);
  declare_parameter("rotation.euler_rad.roll", 0.0);
  declare_parameter("rotation.euler_rad.pitch", 0.0);
  declare_parameter("rotation.euler_rad.yaw", 0.0);
  declare_parameter("rotation.euler_degrees.roll", 0.0);
  declare_parameter("rotation.euler_degrees.pitch", 0.0);
  declare_parameter("rotation.euler_degrees.yaw", 0.0);
  declare_parameter("use_quaternion", true);
  declare_parameter("use_euler_rad", false);
  declare_parameter("use_euler_degrees", false);

  need_to_refresh_msg_ = true;

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TfFixed::timer_callback, this)
  );

  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TfFixed::on_set_parameters_callback, this, std::placeholders::_1)
  );
}

void TfFixed::timer_callback()
{
  if (need_to_refresh_msg_) {
    refresh_msg();
  }
  tf_fixed_msg_->header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(*tf_fixed_msg_);
}

rcl_interfaces::msg::SetParametersResult TfFixed::on_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    switch (hash(param.get_name().c_str())) {
      case hash("use_quaternion"):
      case hash("use_euler_rad"):
      case hash("use_euler_degrees"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          RCLCPP_WARN(this->get_logger(), "Invalid type for %s. Expected boolean.", param.get_name().c_str());
          result.successful = false;
        } else {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("translation.x"):
      case hash("translation.y"):
      case hash("translation.z"):
      case hash("rotation.quaternion.x"):
      case hash("rotation.quaternion.y"):
      case hash("rotation.quaternion.z"):
      case hash("rotation.quaternion.w"):
      case hash("rotation.euler_rad.roll"):
      case hash("rotation.euler_rad.pitch"):
      case hash("rotation.euler_rad.yaw"):
      case hash("rotation.euler_degrees.roll"):
      case hash("rotation.euler_degrees.pitch"):
      case hash("rotation.euler_degrees.yaw"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected double.");
          result.successful = false;
        } else {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("frame_id"):
      case hash("child_frame_id"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected string.");
          result.successful = false;
        } else {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("period_ms"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected integer.");
          result.successful = false;
        }
        if (param.get_value<int64_t>() <= 0) {
          RCLCPP_WARN(this->get_logger(), "Invalid value for period_ms. Expected positive integer.");
          result.successful = false;
        } else {
          reset_timer_period(param.get_value<int64_t>());
        }
        break;
      default:
        RCLCPP_WARN_STREAM(this->get_logger(), "Unknown parameter " << param.get_name());
        break;
    }
  }
  return result;
}

void TfFixed::reset_timer_period(const int64_t &new_period_ms)
{
  if (new_period_ms <= 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid period_ms value: " << new_period_ms << ". Must be positive.");
    return;
  }
  timer_->cancel();
  timer_->reset();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(new_period_ms),
    std::bind(&TfFixed::timer_callback, this)
  );
}

void TfFixed::refresh_msg()
{
  tf_fixed_msg_->header.frame_id = this->get_parameter("frame_id").as_string();
  tf_fixed_msg_->child_frame_id = this->get_parameter("child_frame_id").as_string();

  tf_fixed_msg_->transform.translation.x = this->get_parameter("translation.x").as_double();
  tf_fixed_msg_->transform.translation.y = this->get_parameter("translation.y").as_double();
  tf_fixed_msg_->transform.translation.z = this->get_parameter("translation.z").as_double();

  if (this->get_parameter("use_quaternion").as_bool()) {
    tf_fixed_msg_->transform.rotation.x = this->get_parameter("rotation.quaternion.x").as_double();
    tf_fixed_msg_->transform.rotation.y = this->get_parameter("rotation.quaternion.y").as_double();
    tf_fixed_msg_->transform.rotation.z = this->get_parameter("rotation.quaternion.z").as_double();
    tf_fixed_msg_->transform.rotation.w = this->get_parameter("rotation.quaternion.w").as_double();
  } else if (this->get_parameter("use_euler_rad").as_bool()) {
    double roll = this->get_parameter("rotation.euler_rad.roll").as_double();
    double pitch = this->get_parameter("rotation.euler_rad.pitch").as_double();
    double yaw = this->get_parameter("rotation.euler_rad.yaw").as_double();
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf_fixed_msg_->transform.rotation.x = q.x();
    tf_fixed_msg_->transform.rotation.y = q.y();
    tf_fixed_msg_->transform.rotation.z = q.z();
    tf_fixed_msg_->transform.rotation.w = q.w();
  } else if (this->get_parameter("use_euler_degrees").as_bool()) {
    double roll_deg = this->get_parameter("rotation.euler_degrees.roll").as_double();
    double pitch_deg = this->get_parameter("rotation.euler_degrees.pitch").as_double();
    double yaw_deg = this->get_parameter("rotation.euler_degrees.yaw").as_double();
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);
    tf_fixed_msg_->transform.rotation.x = q.x();
    tf_fixed_msg_->transform.rotation.y = q.y();
    tf_fixed_msg_->transform.rotation.z = q.z();
    tf_fixed_msg_->transform.rotation.w = q.w();
  } else {
    // Default to identity rotation
    tf_fixed_msg_->transform.rotation.x = 0.0;
    tf_fixed_msg_->transform.rotation.y = 0.0;
    tf_fixed_msg_->transform.rotation.z = 0.0;
    tf_fixed_msg_->transform.rotation.w = 1.0;
  }

  need_to_refresh_msg_ = false;
}

} // namespace tf_fixed

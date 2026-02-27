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
        std::bind(&TfFixed::timer_callback, this));

    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TfFixed::on_set_parameters_callback, this, std::placeholders::_1));
  }

  void TfFixed::timer_callback()
  {
    if (need_to_refresh_msg_)
    {
      refresh_msg();
    }
    tf_fixed_msg_->header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(*tf_fixed_msg_);
  }

  rcl_interfaces::msg::SetParametersResult TfFixed::on_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      switch (hash(param.get_name().c_str()))
      {
      case hash("use_quaternion"):
      case hash("use_euler_rad"):
      case hash("use_euler_degrees"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          RCLCPP_WARN(this->get_logger(), "Invalid type for %s. Expected boolean.", param.get_name().c_str());
          result.successful = false;
        }
        else
        {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("translation.x"):
      case hash("translation.y"):
      case hash("translation.z"):
        check_valid_double_parameter(param, result);
        if (result.successful) need_to_refresh_msg_ = true;
        break;
      case hash("rotation.quaternion.x"):
      case hash("rotation.quaternion.y"):
      case hash("rotation.quaternion.z"):
      case hash("rotation.quaternion.w"):
        check_valid_double_parameter(param, result);
        if (result.successful && get_parameter("use_quaternion").as_bool())
        {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("rotation.euler_rad.roll"):
      case hash("rotation.euler_rad.pitch"):
      case hash("rotation.euler_rad.yaw"):
        check_valid_double_parameter(param, result);
        if (result.successful && get_parameter("use_euler_rad").as_bool())
        {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("rotation.euler_degrees.roll"):
      case hash("rotation.euler_degrees.pitch"):
      case hash("rotation.euler_degrees.yaw"):
        check_valid_double_parameter(param, result);
        if (result.successful && get_parameter("use_euler_degrees").as_bool())
        {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("frame_id"):
      case hash("child_frame_id"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected string.");
          result.successful = false;
        }
        else
        {
          need_to_refresh_msg_ = true;
        }
        break;
      case hash("period_ms"):
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected integer.");
          result.successful = false;
        }
        else if (param.get_value<int64_t>() <= 0)
        {
          RCLCPP_WARN(this->get_logger(), "Invalid value for period_ms. Expected positive integer.");
          result.successful = false;
        }
        else
        {
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
    if (new_period_ms <= 0)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid period_ms value: " << new_period_ms << ". Must be positive.");
      return;
    }
    timer_->cancel();
    timer_->reset();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(new_period_ms),
        std::bind(&TfFixed::timer_callback, this));
  }

  void TfFixed::refresh_msg()
  {
    tf_fixed_msg_->header.frame_id = this->get_parameter("frame_id").as_string();
    tf_fixed_msg_->child_frame_id = this->get_parameter("child_frame_id").as_string();

    tf_fixed_msg_->transform.translation.x = this->get_parameter("translation.x").as_double();
    tf_fixed_msg_->transform.translation.y = this->get_parameter("translation.y").as_double();
    tf_fixed_msg_->transform.translation.z = this->get_parameter("translation.z").as_double();

    tf2::Quaternion q;
    double roll_rad, pitch_rad, yaw_rad;
    double roll_deg, pitch_deg, yaw_deg;

    if (this->get_parameter("use_quaternion").as_bool())
    {
      q.setValue(
        this->get_parameter("rotation.quaternion.x").as_double(),
        this->get_parameter("rotation.quaternion.y").as_double(),
        this->get_parameter("rotation.quaternion.z").as_double(),
        this->get_parameter("rotation.quaternion.w").as_double()
      );
      tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);
      roll_deg = roll_rad * 180.0 / M_PI;
      pitch_deg = pitch_rad * 180.0 / M_PI;
      yaw_deg = yaw_rad * 180.0 / M_PI;

      this->set_parameters_atomically({
        rclcpp::Parameter("rotation.euler_rad.roll", roll_rad),
        rclcpp::Parameter("rotation.euler_rad.pitch", pitch_rad),
        rclcpp::Parameter("rotation.euler_rad.yaw", yaw_rad),
        rclcpp::Parameter("rotation.euler_degrees.roll", roll_deg),
        rclcpp::Parameter("rotation.euler_degrees.pitch", pitch_deg),
        rclcpp::Parameter("rotation.euler_degrees.yaw", yaw_deg)
      });
    }
    else if (this->get_parameter("use_euler_rad").as_bool())
    {
      roll_rad = this->get_parameter("rotation.euler_rad.roll").as_double();
      pitch_rad = this->get_parameter("rotation.euler_rad.pitch").as_double();
      yaw_rad = this->get_parameter("rotation.euler_rad.yaw").as_double();
      q.setRPY(roll_rad, pitch_rad, yaw_rad);
      roll_deg = roll_rad * 180.0 / M_PI;
      pitch_deg = pitch_rad * 180.0 / M_PI;
      yaw_deg = yaw_rad * 180.0 / M_PI;

      this->set_parameters_atomically({
        rclcpp::Parameter("rotation.quaternion.x", q.x()),
        rclcpp::Parameter("rotation.quaternion.y", q.y()),
        rclcpp::Parameter("rotation.quaternion.z", q.z()),
        rclcpp::Parameter("rotation.quaternion.w", q.w()),
        rclcpp::Parameter("rotation.euler_degrees.roll", roll_deg),
        rclcpp::Parameter("rotation.euler_degrees.pitch", pitch_deg),
        rclcpp::Parameter("rotation.euler_degrees.yaw", yaw_deg)
      });
    }
    else if (this->get_parameter("use_euler_degrees").as_bool())
    {
      roll_deg = this->get_parameter("rotation.euler_degrees.roll").as_double();
      pitch_deg = this->get_parameter("rotation.euler_degrees.pitch").as_double();
      yaw_deg = this->get_parameter("rotation.euler_degrees.yaw").as_double();
      q.setRPY(roll_rad, pitch_rad, yaw_rad);
      roll_rad = roll_deg * M_PI / 180.0;
      pitch_rad = pitch_deg * M_PI / 180.0;
      yaw_rad = yaw_deg * M_PI / 180.0;

      this->set_parameters_atomically({
        rclcpp::Parameter("rotation.quaternion.x", q.x()),
        rclcpp::Parameter("rotation.quaternion.y", q.y()),
        rclcpp::Parameter("rotation.quaternion.z", q.z()),
        rclcpp::Parameter("rotation.quaternion.w", q.w()),
        rclcpp::Parameter("rotation.euler_rad.roll", roll_rad),
        rclcpp::Parameter("rotation.euler_rad.pitch", pitch_rad),
        rclcpp::Parameter("rotation.euler_rad.yaw", yaw_rad)
      });
    }
    else
    {
      // Default to identity rotation
      q.setValue(0.0, 0.0, 0.0, 1.0);
      this->set_parameters_atomically({
        rclcpp::Parameter("rotation.quaternion.x", 0.0),
        rclcpp::Parameter("rotation.quaternion.y", 0.0),
        rclcpp::Parameter("rotation.quaternion.z", 0.0),
        rclcpp::Parameter("rotation.quaternion.w", 1.0),
        rclcpp::Parameter("rotation.euler_rad.roll", 0.0),
        rclcpp::Parameter("rotation.euler_rad.pitch", 0.0),
        rclcpp::Parameter("rotation.euler_rad.yaw", 0.0),
        rclcpp::Parameter("rotation.euler_degrees.roll", 0.0),
        rclcpp::Parameter("rotation.euler_degrees.pitch", 0.0),
        rclcpp::Parameter("rotation.euler_degrees.yaw", 0.0)
      });
    }

    tf_fixed_msg_->transform.rotation.x = q.x();
    tf_fixed_msg_->transform.rotation.y = q.y();
    tf_fixed_msg_->transform.rotation.z = q.z();
    tf_fixed_msg_->transform.rotation.w = q.w();

    need_to_refresh_msg_ = false;
  }

  void TfFixed::check_valid_double_parameter(const rclcpp::Parameter &param, rcl_interfaces::msg::SetParametersResult &result)
  {
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type for " << param.get_name() << ". Expected double.");
      result.successful = false;
    }
    else if (std::isnan(param.get_value<double>()) || std::isinf(param.get_value<double>()))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid value for " << param.get_name() << ". Expected finite number.");
      result.successful = false;
    }
  }

} // namespace tf_fixed

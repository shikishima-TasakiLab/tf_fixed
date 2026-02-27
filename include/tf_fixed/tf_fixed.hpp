#ifndef TF_FIXED_HPP
#define TF_FIXED_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tf_fixed
{

constexpr unsigned int hash(const char * str) {
    return *str ? static_cast<unsigned int>(*str) + 33 * hash(str + 1) : 5381;
}

class TfFixed : public rclcpp::Node
{
public:
    TfFixed();

protected:
    void timer_callback();
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
    void reset_timer_period(const int64_t &new_period_ms);
    void refresh_msg();
    void check_valid_double_parameter(const rclcpp::Parameter &param, rcl_interfaces::msg::SetParametersResult &result);

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped::SharedPtr tf_fixed_msg_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
    bool need_to_refresh_msg_ = false;

};

} // namespace tf_fixed

#endif // TF_FIXED_HPP

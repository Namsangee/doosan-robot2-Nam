#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tcp_force_broadcaster
{

class TcpForceBroadcaster : public controller_interface::ControllerInterface
{
public:
  TcpForceBroadcaster() = default;

  controller_interface::CallbackReturn on_init() override
  {
    frame_id_ = auto_declare<std::string>("frame_id", "tcp");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        "tcp_force/force_0",
        "tcp_force/force_1",
        "tcp_force/force_2",
        "tcp_force/force_3",
        "tcp_force/force_4",
        "tcp_force/force_5"
      }
    };
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    force_pub_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/tcp_force", rclcpp::SystemDefaultsQoS());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = time;
    msg.header.frame_id = frame_id_;

    msg.wrench.force.x  = state_interfaces_[0].get_value();
    msg.wrench.force.y  = state_interfaces_[1].get_value();
    msg.wrench.force.z  = state_interfaces_[2].get_value();
    msg.wrench.torque.x = state_interfaces_[3].get_value();
    msg.wrench.torque.y = state_interfaces_[4].get_value();
    msg.wrench.torque.z = state_interfaces_[5].get_value();

    force_pub_->publish(msg);
    return controller_interface::return_type::OK;
  }

private:
  std::string frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
};

}  // namespace tcp_force_broadcaster

PLUGINLIB_EXPORT_CLASS(
  tcp_force_broadcaster::TcpForceBroadcaster,
  controller_interface::ControllerInterface)

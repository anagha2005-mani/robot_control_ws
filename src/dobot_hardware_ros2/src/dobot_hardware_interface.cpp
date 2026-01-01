#include "dobot_hardware/dobot_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dobot_hardware
{

bool DobotHardware::connectDevice()
{
  device_connected_ = false;
  // TODO: Add Dobot SDK connection here
  device_connected_ = true;
  return device_connected_;
}

hardware_interface::CallbackReturn DobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DobotHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  device_connected_ = connectDevice();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  device_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DobotHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Mirror commands to state (for now)
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!device_connected_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // TODO: Send hw_commands_ to Dobot using SDK
  return hardware_interface::return_type::OK;
}

}  // namespace dobot_hardware

PLUGINLIB_EXPORT_CLASS(
  dobot_hardware::DobotHardware,
  hardware_interface::SystemInterface
)

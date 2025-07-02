#ifndef MY_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
#define MY_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <serial/serial.h>

namespace my_hardware_interface
{
class RRBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 原有成員變量
  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;
  std::vector<double> joint_states_;

  // UART 相關成員變量
  std::unique_ptr<serial::Serial> serial_port_;
  std::string uart_port_;
  int uart_baudrate_;
  std::atomic<bool> uart_connected_;
  
  // UART 初始化和清理
  bool initializeUART();
  void closeUART();
  std::string readUARTData();
};

}  // namespace my_hardware_interface

#endif  // MY_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

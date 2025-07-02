// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "my_hardware_interface/rrbot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_hardware_interface
{
hardware_interface::CallbackReturn RRBotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 讀取 UART 參數
  uart_port_ = info_.hardware_parameters.count("uart_port") ? 
               info_.hardware_parameters.at("uart_port") : "/dev/ttyUSB0";
  uart_baudrate_ = info_.hardware_parameters.count("uart_baudrate") ? 
                   std::stoi(info_.hardware_parameters.at("uart_baudrate")) : 9600;

  RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), 
              "UART Port: %s, Baudrate: %d", uart_port_.c_str(), uart_baudrate_);

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  uart_connected_ = false;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // 初始化 UART 連接
  if (!initializeUART())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), "Failed to initialize UART connection");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RRBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    hw_states_[i]=stod(info_.joints[i].state_interfaces[0].initial_value);
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RRBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  hw_commands_ = hw_states_;
  joint_states_ = hw_states_; 
  
  RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Hardware interface activated");
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  closeUART();
  
  RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Hardware interface deactivated");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  hw_states_ = joint_states_;

  // 讀取 STM32 UART 數據
  if (uart_connected_)
  {
    std::string uart_data = readUARTData();
    if (!uart_data.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), 
                  "Received from STM32: %s", uart_data.c_str());
      printf("STM32 Data: %s\n", uart_data.c_str());
    }
  }
  else
  {
     printf("stm not connect ");
  }
  /*
  printf("Joint States - ");
  for(auto i = 0ul; i < hw_states_.size(); i++)
  {
    printf("Joint %lu: %f ", i, hw_states_[i]);
  }
  printf("\n");
  */

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'
  joint_states_ = hw_commands_;
  /*
  printf("Commands - ");
  for(auto i = 0ul; i < hw_commands_.size(); i++)
  {
    printf("Joint %lu: %f ", i, hw_commands_[i]);
  }
  printf("\n");
  */
  return hardware_interface::return_type::OK;
}

bool RRBotHardwareInterface::initializeUART()
{
  try
  {
    serial_port_ = std::make_unique<serial::Serial>(uart_port_, uart_baudrate_, 
                                                   serial::Timeout::simpleTimeout(1000));
    
    if (serial_port_->isOpen())
    {
      uart_connected_ = true;
      RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), 
                  "UART connection established on %s at %d baud", 
                  uart_port_.c_str(), uart_baudrate_);
      return true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), 
                   "Failed to open UART port %s", uart_port_.c_str());
      return false;
    }
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), 
                 "UART IOException: %s", e.what());
    return false;
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), 
                 "UART Exception: %s", e.what());
    return false;
  }
}

void RRBotHardwareInterface::closeUART()
{
  if (serial_port_ && serial_port_->isOpen())
  {
    serial_port_->close();
    uart_connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "UART connection closed");
  }
}

std::string RRBotHardwareInterface::readUARTData()
{
  if (!uart_connected_ || !serial_port_->isOpen()) {
    return "";
  }

  try {
    size_t available = serial_port_->available();
    if (available > 0) {
      // 直接讀取所有可用數據
      return serial_port_->read(available);
    }
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), 
                 "UART Read IOException: %s", e.what());
    uart_connected_ = false;
  } catch (std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RRBotHardwareInterface"), 
                 "UART Read Exception: %s", e.what());
  }

  return "";
}



}  // namespace my_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_hardware_interface::RRBotHardwareInterface, hardware_interface::SystemInterface)

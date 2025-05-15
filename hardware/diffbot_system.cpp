// Copyright 2021 ros2_control Development Team
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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info.hardware_parameters.at("device");
  baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  timeout_ms_ = std::stoi(info.hardware_parameters.at("timeout_ms"));
  
  joint_velocity_command_.resize(2, 0.0);
  joint_position_state_.resize(2, 0.0);
  joint_velocity_state_.resize(2, 0.0);

 
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

try {
    serial_port_ = std::make_shared<boost::asio::serial_port>(io_service_, port_name_);
    serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate_));
    serial_port_->set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));
      serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    io_thread_ = boost::thread([this]() { io_service_.run(); });
    

    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Connected to %s at %d baud", 
                port_name_.c_str(), baud_rate_);
  } catch (const boost::system::system_error &e) {
    RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Port open failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

if (serial_port_) {
    serial_port_->cancel();
    serial_port_->close();
    serial_port_.reset();
  }
  io_service_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
 {
  std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("base_left_wheel_joint", "position", &joint_position_state_[0]);
    state_interfaces.emplace_back("base_left_wheel_joint", "velocity", &joint_velocity_state_[0]);
    state_interfaces.emplace_back("base_right_wheel_joint", "position", &joint_position_state_[1]);
    state_interfaces.emplace_back("base_right_wheel_joint", "velocity", &joint_velocity_state_[1]);

  return state_interfaces;
 }

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("base_left_wheel_joint", "velocity", &joint_velocity_command_[0]);
    command_interfaces.emplace_back("base_right_wheel_joint", "velocity", &joint_velocity_command_[1]);
  return command_interfaces;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
if (!serial_port_ || !serial_port_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  boost::mutex::scoped_lock lock(mutex_);
  std::stringstream cmd;
  cmd << "M " << joint_velocity_command_[0] << " " << joint_velocity_command_[1] << "\n" ;

  try {
    serial_port_->write_some(boost::asio::buffer(cmd.str()));
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)

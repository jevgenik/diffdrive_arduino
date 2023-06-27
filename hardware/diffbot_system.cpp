// Implementation of a hardware interface (for ROS2 control) for a differential drive Arduino-based robot.

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

#include "diffdrive_arduino/diffbot_system.hpp" // path is src/diffdrive_arduino_humble/diffdrive_arduino/include/diffdrive_arduino/diffbot_system.hpp

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

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
    
    // cfg_ is a struct defined in diffbot_system.hpp (is for storing the parameters)
    // info_ is a struct defined in hardware_interface/types/hardware_interface_types.hpp
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"]; 
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }
    

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


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
  
  // Returns a vector of hardware_interface::StateInterface objects. This function is responsible for exporting 
  // the state interfaces of the hardware to be used within the ros2_control framework.
  // By implementing the export_state_interfaces function and providing the necessary state interfaces, 
  // the DiffDriveArduinoHardware class allows the ros2_control framework to access and read the state information 
  // (such as position and velocity) of the wheels controlled by the Arduino-based hardware interface. 
  // This information can then be used by controllers and other components within the ROS ecosystem for robot monitoring.
  // The current implementation supports two state interfaces for each wheel: position and velocity.
  std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces; // vector of hardware_interface::StateInterface objects

    // The hardware_interface::StateInterface is a class
    // eplace_back() is a function that adds an element to the end of the vector
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    return state_interfaces;
  }
 
  // Returns a vector of hardware_interface::CommandInterface objects. This function is responsible for exporting
  // the command interfaces of the hardware to be used within the ros2_control framework.
  // Once the command interfaces are exported and registered with the ros2_control system, controllers can use them 
  // to send commands to the hardware at the desired frequency. 
  // THe current implementation supports one command interface for each wheel: velocity.
  std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // This is called when the state switches to the "activating" state
  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (cfg_.pid_p > 0)
    {
      comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

    double delta_seconds = period.seconds();

    double pos_prev = wheel_l_.pos;
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  // This is called periodically when the state is "active"
  // Function is called by the controller manager or the control loop infrastructure within the ros2_control framework.
  // It is responsible for updating the hardware with the desired control commands based on the current state of the system.
  hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected()) // comms_ is an instance of the ArduinoComms class
                             // that is created in the constructor of the DiffDriveArduinoHardware class
    {
      return hardware_interface::return_type::ERROR;
    }

    int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
    int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
    comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
    return hardware_interface::return_type::OK;
  }

}  // namespace diffdrive_arduino

// This macro exports the component when using the pluginlib/class_loader
// DiffDriveArduinoHardware class becomes discoverable and usable as a (hardware interface) plugin 
// by other components in the ROS ecosystem. It enables the ros2_control framework to dynamically 
// load and instantiate instances of the DiffDriveArduinoHardware class as needed.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)

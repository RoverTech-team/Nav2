// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "ethercat_driver/ethercat_driver.hpp"

#include <tinyxml2.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{

namespace
{

bool has_true_parameter(
  const hardware_interface::ComponentInfo & component,
  const std::string & key)
{
  auto it = component.parameters.find(key);
  return it != component.parameters.end() && it->second == "true";
}

int find_interface_index(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  for (size_t i = 0; i < interfaces.size(); ++i) {
    if (interfaces[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

uint16_t parse_u16(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    throw std::runtime_error("Missing required parameter: " + key);
  }
  const auto value = std::stoul(it->second);
  if (value > std::numeric_limits<uint16_t>::max()) {
    throw std::runtime_error("Parameter out of uint16_t range: " + key);
  }
  return static_cast<uint16_t>(value);
}

}  // namespace

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;
  ec_modules_.clear();
  ec_module_parameters_.clear();

  hw_joint_states_.clear();
  hw_sensor_states_.clear();
  hw_gpio_states_.clear();
  hw_joint_commands_.clear();
  hw_sensor_commands_.clear();
  hw_gpio_commands_.clear();

  hw_joint_states_.resize(info_.joints.size());
  for (size_t j = 0; j < info_.joints.size(); ++j) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());

    const int pos_idx = find_interface_index(info_.joints[j].state_interfaces, "position");
    if (pos_idx >= 0) {
      hw_joint_states_[j][pos_idx] = 0.0;
    }
  }

  hw_sensor_states_.resize(info_.sensors.size());
  for (size_t s = 0; s < info_.sensors.size(); ++s) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  hw_gpio_states_.resize(info_.gpios.size());
  for (size_t g = 0; g < info_.gpios.size(); ++g) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  hw_joint_commands_.resize(info_.joints.size());
  for (size_t j = 0; j < info_.joints.size(); ++j) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  hw_sensor_commands_.resize(info_.sensors.size());
  for (size_t s = 0; s < info_.sensors.size(); ++s) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  hw_gpio_commands_.resize(info_.gpios.size());
  for (size_t g = 0; g < info_.gpios.size(); ++g) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");

    for (size_t i = 0; i < module_params.size(); ++i) {
      module_params[i]["joint_name"] = info_.joints[j].name;

      for (size_t k = 0; k < info_.joints[j].state_interfaces.size(); ++k) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (size_t k = 0; k < info_.joints[j].command_interfaces.size(); ++k) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }

      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j])) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of joint module %zu FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }

        ec_module_parameters_.push_back(module_params[i]);
        ec_modules_.push_back(module);
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Plugin of joint %s failed to load. Error: %s",
          info_.joints[j].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      } catch (const std::exception & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Setup of joint %s failed. Error: %s",
          info_.joints[j].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }
    }
  }

  for (size_t g = 0; g < info_.gpios.size(); ++g) {
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");

    for (size_t i = 0; i < module_params.size(); ++i) {
      for (size_t k = 0; k < info_.gpios[g].state_interfaces.size(); ++k) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (size_t k = 0; k < info_.gpios[g].command_interfaces.size(); ++k) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }

      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g])) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %zu FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }

        ec_module_parameters_.push_back(module_params[i]);
        ec_modules_.push_back(module);
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Plugin of GPIO %s failed to load. Error: %s",
          info_.gpios[g].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      } catch (const std::exception & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Setup of GPIO %s failed. Error: %s",
          info_.gpios[g].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }
    }
  }

  for (size_t s = 0; s < info_.sensors.size(); ++s) {
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");

    for (size_t i = 0; i < module_params.size(); ++i) {
      for (size_t k = 0; k < info_.sensors[s].state_interfaces.size(); ++k) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (size_t k = 0; k < info_.sensors[s].command_interfaces.size(); ++k) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }

      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s])) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of sensor module %zu FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }

        ec_module_parameters_.push_back(module_params[i]);
        ec_modules_.push_back(module);
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Plugin of sensor %s failed to load. Error: %s",
          info_.sensors[s].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      } catch (const std::exception & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Setup of sensor %s failed. Error: %s",
          info_.sensors[s].name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "Got %zu modules", ec_modules_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    for (size_t i = 0; i < info_.joints[j].state_interfaces.size(); ++i) {
      state_interfaces.emplace_back(
        info_.joints[j].name,
        info_.joints[j].state_interfaces[i].name,
        &hw_joint_states_[j][i]);
    }
  }

  for (size_t s = 0; s < info_.sensors.size(); ++s) {
    for (size_t i = 0; i < info_.sensors[s].state_interfaces.size(); ++i) {
      state_interfaces.emplace_back(
        info_.sensors[s].name,
        info_.sensors[s].state_interfaces[i].name,
        &hw_sensor_states_[s][i]);
    }
  }

  for (size_t g = 0; g < info_.gpios.size(); ++g) {
    for (size_t i = 0; i < info_.gpios[g].state_interfaces.size(); ++i) {
      state_interfaces.emplace_back(
        info_.gpios[g].name,
        info_.gpios[g].state_interfaces[i].name,
        &hw_gpio_states_[g][i]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    for (size_t i = 0; i < info_.joints[j].command_interfaces.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[j].name,
        info_.joints[j].command_interfaces[i].name,
        &hw_joint_commands_[j][i]);
    }
  }

  for (size_t s = 0; s < info_.sensors.size(); ++s) {
    for (size_t i = 0; i < info_.sensors[s].command_interfaces.size(); ++i) {
      command_interfaces.emplace_back(
        info_.sensors[s].name,
        info_.sensors[s].command_interfaces[i].name,
        &hw_sensor_commands_[s][i]);
    }
  }

  for (size_t g = 0; g < info_.gpios.size(); ++g) {
    for (size_t i = 0; i < info_.gpios[g].command_interfaces.size(); ++i) {
      command_interfaces.emplace_back(
        info_.gpios[g].name,
        info_.gpios[g].command_interfaces[i].name,
        &hw_gpio_commands_[g][i]);
    }
  }

  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);

  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ... please wait ...");

  try {
    if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
      control_frequency_ = 100.0;
    } else {
      control_frequency_ = std::stod(info_.hardware_parameters.at("control_frequency"));
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Invalid control_frequency parameter: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  if (control_frequency_ <= 0.0) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Invalid control frequency");
    return CallbackReturn::ERROR;
  }

  master_.setCtrlFrequency(control_frequency_);

  try {
    for (size_t i = 0; i < ec_modules_.size(); ++i) {
      const uint16_t alias = parse_u16(ec_module_parameters_[i], "alias");
      const uint16_t position = parse_u16(ec_module_parameters_[i], "position");
      master_.addSlave(alias, position, ec_modules_[i].get());
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Failed to add EtherCAT slave: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < ec_modules_.size(); ++i) {
    uint16_t position = 0;
    try {
      position = parse_u16(ec_module_parameters_[i], "position");
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid module position: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    for (const auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code = 0;
      const int ret = master_.configSlaveSdo(position, sdo, &abort_code);
      if (ret != 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module at position %u, abort code: 0x%08x",
          position, abort_code);
        return CallbackReturn::ERROR;
      }
    }
  }

  if (!master_.activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster");

  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec += 1;

  constexpr int max_startup_cycles = 500;
  for (int cycle = 0; cycle < max_startup_cycles; ++cycle) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);

    master_.update();

    bool is_all_init = true;
    for (const auto & module : ec_modules_) {
      is_all_init = is_all_init && module->initialized();
    }

    if (is_all_init) {
      activated_ = true;
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System successfully started");
      return CallbackReturn::SUCCESS;
    }

    t.tv_nsec += master_.getInterval();
    while (t.tv_nsec >= 1000000000L) {
      t.tv_nsec -= 1000000000L;
      ++t.tv_sec;
    }
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("EthercatDriver"),
    "Timeout while waiting for EtherCAT modules to initialize");

  master_.stop();
  activated_ = false;
  return CallbackReturn::ERROR;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ... please wait ...");
  master_.stop();
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System successfully stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !activated_) {
    return hardware_interface::return_type::OK;
  }

  master_.readData();

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    if (has_true_parameter(info_.joints[j], "invert_velocity")) {
      const int vel_idx = find_interface_index(info_.joints[j].state_interfaces, "velocity");
      if (vel_idx >= 0 && std::isfinite(hw_joint_states_[j][vel_idx])) {
        hw_joint_states_[j][vel_idx] = -hw_joint_states_[j][vel_idx];
      }
    }
  }

  const double dt = period.seconds();
  if (dt > 0.0) {
    for (size_t j = 0; j < info_.joints.size(); ++j) {
      const int pos_idx = find_interface_index(info_.joints[j].state_interfaces, "position");
      const int vel_idx = find_interface_index(info_.joints[j].state_interfaces, "velocity");

      if (pos_idx >= 0 && vel_idx >= 0) {
        const double vel = hw_joint_states_[j][vel_idx];
        if (std::isfinite(vel)) {
          if (!std::isfinite(hw_joint_states_[j][pos_idx])) {
            hw_joint_states_[j][pos_idx] = 0.0;
          }
          hw_joint_states_[j][pos_idx] += vel * dt;
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !activated_) {
    return hardware_interface::return_type::OK;
  }

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    const bool invert = has_true_parameter(info_.joints[j], "invert_velocity");

    for (size_t i = 0; i < hw_joint_commands_[j].size(); ++i) {
      double cmd = hw_joint_commands_[j][i];

      if (!std::isfinite(cmd)) {
        cmd = 0.0;
      }

      if (
        invert &&
        i < info_.joints[j].command_interfaces.size() &&
        info_.joints[j].command_interfaces[i].name == "velocity")
      {
        cmd = -cmd;
      }

      hw_joint_commands_[j][i] = cmd;
    }
  }

  master_.writeData();
  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>>
EthercatDriver::getEcModuleParam(
  std::string & urdf,
  std::string component_name,
  std::string component_type)
{
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }

  tinyxml2::XMLDocument doc;
  const tinyxml2::XMLError parse_result = doc.Parse(urdf.c_str());
  if (parse_result != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("invalid URDF passed into robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (!robot_it || std::string(robot_it->Name()) != "robot") {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;

  while (ros2_control_it) {
    const auto * ros2_control_child_it =
      ros2_control_it->FirstChildElement(component_type.c_str());

    while (ros2_control_child_it) {
      const char * child_name = ros2_control_child_it->Attribute("name");
      if (child_name != nullptr && component_name == child_name) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");

        while (ec_module_it) {
          std::unordered_map<std::string, std::string> module_param;

          if (const char * module_name = ec_module_it->Attribute("name")) {
            module_param["name"] = module_name;
          }

          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (plugin_it != nullptr && plugin_it->GetText() != nullptr) {
            module_param["plugin"] = plugin_it->GetText();
          }

          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            const char * param_name = param_it->Attribute("name");
            const char * param_text = param_it->GetText();
            if (param_name != nullptr && param_text != nullptr) {
              module_param[param_name] = param_text;
            }
            param_it = param_it->NextSiblingElement("param");
          }

          if (module_param.find("plugin") == module_param.end()) {
            throw std::runtime_error("ec_module is missing required <plugin>");
          }
          if (module_param.find("alias") == module_param.end()) {
            throw std::runtime_error("ec_module is missing required alias parameter");
          }
          if (module_param.find("position") == module_param.end()) {
            throw std::runtime_error("ec_module is missing required position parameter");
          }

          module_params.push_back(std::move(module_param));
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }

      ros2_control_child_it =
        ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }

    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

}  // namespace ethercat_driver

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver,
  hardware_interface::SystemInterface)

// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace rio
{
enum class RosParameterType
{
  Required,
  Recommended,
  Optional
};

template <typename T>
static bool getRosParameter(const rclcpp::Node::SharedPtr& nh,
                            const std::string& kPrefix,
                            const RosParameterType& param_type,
                            const std::string& param_name,
                            T& param)
{
  // 1. Prüfen, ob der Parameter bereits deklariert wurde oder durch den Nutzer 
  // (via Launch-File oder Terminal) überschrieben wurde.
  bool is_configured = false;
  auto overrides = nh->get_node_parameters_interface()->get_parameter_overrides();
  
  if (overrides.find(param_name) != overrides.end() || nh->has_parameter(param_name)) {
    is_configured = true;
  }

  // 2. Warnungen ausgeben, falls der Parameter fehlt
  if (!is_configured)
  {
    if (param_type == RosParameterType::Optional)
    {
      RCLCPP_INFO_STREAM(nh->get_logger(), kPrefix << "<" << param_name
                              << "> is optional but not configured. Using default value: " << param);
    }
    else if (param_type == RosParameterType::Recommended)
    {
      RCLCPP_WARN_STREAM(nh->get_logger(), kPrefix << "<" << param_name
                              << "> is recommended but not configured. Using default value: " << param);
    }
    else
    {
      RCLCPP_ERROR_STREAM(nh->get_logger(), kPrefix << "<" << param_name << "> is required but not configured. Exiting!");
      return false;
    }
  }

  // 3. In ROS 2 MÜSSEN Parameter deklariert werden. 
  // Wenn der Parameter nicht übergeben wurde, wird er hier mit dem Default-Wert aus 'param' initialisiert.
  if (!nh->has_parameter(param_name)) {
    nh->declare_parameter<T>(param_name, param);
  }

  // 4. Den finalen Wert in die Referenz-Variable schreiben
  nh->get_parameter(param_name, param);

  return true;
}

// bool getRosParameter(const rclcpp::Node::SharedPtr& nh, const std::string& param_name, const T& default_value, T& param)
// {
//     // In ROS 2 we declare and get parameters. 
//     // If it's already declared, we just get it.
//     if (!nh->has_parameter(param_name)) {
//         nh->declare_parameter<T>(param_name, default_value);
//     }
//     return nh->get_parameter(param_name, param);
// }

}  // namespace rio

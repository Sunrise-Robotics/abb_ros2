// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
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

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <future>

// Tracing
#ifdef TRACING_ENABLED
# include "abb_hardware_interface/hardware_interface_tp.h"
#endif

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemHardware");

CallbackReturn ABBSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Print the name of the hardware interface
  RCLCPP_INFO_STREAM(LOGGER, "Hardware interface name: " << info_.name);

  // Initialize the is_activated_ flag
  is_activated_ = false;

  // Define the initial joints variable
  std::vector<abb::robot::InitialJointValue> initial_joint_values;

  // Validate interfaces configured in ros2_control xacro.
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // By default, construct the robot_controller_description_ by connecting to RWS.
  // If configure_via_rws is set to false, configure the robot_controller_description_
  // relying on joint information in the ros2_control xacro.
  const auto configure_it = info_.hardware_parameters.find("configure_via_rws");
  const bool configure_via_rws = configure_it == info_.hardware_parameters.end()                    ? true :
                                 configure_it->second == "false" || configure_it->second == "False" ? false :
                                                                                                      true;

  // Initialize a clock so that we can throttle the logging
  clock_ = rclcpp::Clock(RCL_ROS_TIME);

  const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
  const auto rws_ip = info_.hardware_parameters["rws_ip"];
  if (configure_via_rws)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from RWS.");

    if (rws_ip == "None")
    {
      RCLCPP_FATAL(LOGGER, "RWS IP not specified");
      return CallbackReturn::ERROR;
    }

    // Get robot controller description from RWS
    abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
    robot_controller_description_ = abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", true);
  }
  else
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from HardwareInfo.");

    // Add header.
    auto header{ robot_controller_description_.mutable_header() };
    // Omnicore controllers have RobotWare version >=7.0.0.
    header->mutable_robot_ware_version()->set_major_number(7);
    header->mutable_robot_ware_version()->set_minor_number(3);
    header->mutable_robot_ware_version()->set_patch_number(2);

    // Add system indicators.
    auto system_indicators{ robot_controller_description_.mutable_system_indicators() };
    system_indicators->mutable_options()->set_egm(true);

    // Add single mechanical units group.
    auto mug{ robot_controller_description_.add_mechanical_units_groups() };
    mug->set_name("");

    // Add single robot to mechanical units group.
    auto robot{ mug->mutable_robot() };
    robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
    robot->set_axes_total(info_.joints.size());
    robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

    // Add joints to robot.
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const hardware_interface::ComponentInfo& joint = info_.joints[i];
      // We assume it's a revolute joint unless explicitly specified.
      // Check if a "type" key is present in joint.parameters with value other than "revolute"
      // as per sdformat conventions http://sdformat.org/spec?elem=joint.
      const auto type_it = joint.parameters.find("type");
      const bool is_revolute = type_it != joint.parameters.end() && type_it->second != "revolute" ? false : true;

      // Get the range of the joint from its command interfaces.
      for (const hardware_interface::InterfaceInfo& joint_info : joint.command_interfaces)
      {
        if (joint_info.name == hardware_interface::HW_IF_POSITION)
        {
          const double min = std::stod(joint_info.min);
          const double max = std::stod(joint_info.max);

          abb::robot::StandardizedJoint* p_joint = robot->add_standardized_joints();
          p_joint->set_standardized_name(joint.name);
          p_joint->set_rotating_move(is_revolute);
          p_joint->set_lower_joint_bound(min);
          p_joint->set_upper_joint_bound(max);

          RCLCPP_INFO(LOGGER, "Configured component %s of type %s with range [%.3f, %.3f]", joint.name.c_str(),
                      joint.type.c_str(), min, max);
          break;
        }
      }
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                                 << abb::robot::summaryText(robot_controller_description_));

  abb::robot::InitialJointValue initial_value;

  // Get initial joint values from controller using RWS and our custom function
  try
  {
    RCLCPP_INFO(LOGGER, "Getting initial joint values from controller...");
    initial_joint_values = abb::robot::utilities::getRWSJointsFromController(rws_ip, rws_port);
    RCLCPP_INFO(LOGGER, "Successfully retrieved initial joint values");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get initial joint values: " << e.what());
    return CallbackReturn::ERROR;
  }

  // Configure EGM
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

  // Initialize motion data from robot controller description
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_, initial_joint_values);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  // Loop through the joints and print the current joint values
  for (const auto& joint : motion_data_.groups[0].units[0].joints)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Joint state " << joint.name << " has position " << joint.state.position << " and velocity " << joint.state.velocity);
    RCLCPP_INFO_STREAM(LOGGER, "Joint command " << joint.name << " has position " << joint.command.position << " and velocity " << joint.command.velocity);
  }

  // Create channel configuration for each mechanical unit group
  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto& group : robot_controller_description_.mechanical_units_groups())
  {
    try
    {
      const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
      const auto channel_configuration =
          abb::robot::EGMManager::ChannelConfiguration{ static_cast<uint16_t>(egm_port), group };
      channel_configurations.emplace_back(channel_configuration);
      RCLCPP_INFO_STREAM(LOGGER,
                         "Configuring EGM for mechanical unit group " << group.name() << " on port " << egm_port);
    }
    catch (std::invalid_argument& e)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                                                                          << "\" not specified in hardware parameters");
      return CallbackReturn::ERROR;
    }
  }
  try
  {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  try
  {
    // Try to read the current values from the robot
    egm_manager_->read(motion_data_);
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to read the current values from the robot");
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ABBSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        const auto joint_name = joint.name;
        RCLCPP_INFO_STREAM(LOGGER, "Exporting state interface for joint " << joint_name);
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        const auto joint_name = joint.name;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
      }
    }
  }

  return command_interfaces;
}

CallbackReturn ABBSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  size_t counter = 0;
  std::vector<abb::robot::InitialJointValue> initial_joint_values;

  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && ++counter < NUM_CONNECTION_TRIES)
  {
    // Wait for a message on any of the configured EGM channels.
    if (egm_manager_->waitForMessage(500))
    {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    }

    RCLCPP_INFO(LOGGER, "Not connected to robot...");
    if (counter == NUM_CONNECTION_TRIES)
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
      return CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(500ms);
  }

  // Print the joint current and command values from motion_data_
  for (const auto& joint : motion_data_.groups[0].units[0].joints)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Joint state " << joint.name << " has position " << joint.state.position << " and velocity " << joint.state.velocity);
    RCLCPP_DEBUG_STREAM(LOGGER, "Joint command " << joint.name << " has position " << joint.command.position << " and velocity " << joint.command.velocity);
  }

  egm_manager_->read(motion_data_);

  // Get initial joint values from controller using RWS and our custom function
  try
  {
    RCLCPP_DEBUG(LOGGER, "Getting initial joint values from controller...");
    initial_joint_values = abb::robot::utilities::getRWSJointsFromController(info_.hardware_parameters["rws_ip"], stoi(info_.hardware_parameters["rws_port"]));
    RCLCPP_DEBUG(LOGGER, "Successfully retrieved initial joint values");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get initial joint values: " << e.what());
    return CallbackReturn::ERROR;
  }

  // Assign the initial joint values to the motion_data_
  for (size_t i = 0; i < initial_joint_values.size(); ++i)
  {
    motion_data_.groups[0].units[0].joints[i].state.position = initial_joint_values[i].position_state;
    motion_data_.groups[0].units[0].joints[i].state.velocity = initial_joint_values[i].velocity_state;
    motion_data_.groups[0].units[0].joints[i].command.position = initial_joint_values[i].position_command;
    motion_data_.groups[0].units[0].joints[i].command.velocity = initial_joint_values[i].velocity_command;
  }

  // Print the joint values from motion_data_
  for (const auto& joint : motion_data_.groups[0].units[0].joints)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Joint state " << joint.name << " has position " << joint.state.position << " and velocity " << joint.state.velocity);
    RCLCPP_DEBUG_STREAM(LOGGER, "Joint command " << joint.name << " has position " << joint.command.position << " and velocity " << joint.command.velocity);
  }

  is_activated_ = true;

  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully stopped!");
  is_activated_ = false;
  return CallbackReturn::SUCCESS;
}

return_type ABBSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
#ifdef TRACING_ENABLED
  tracepoint(hardware_interface, read_start, info_.name.c_str());
#endif

  egm_manager_->read(motion_data_);
  RCLCPP_DEBUG_THROTTLE(LOGGER, clock_, 1000, "Reading from robot");

#ifdef TRACING_ENABLED
  tracepoint(hardware_interface, read_end, info_.name.c_str());
#endif
  return return_type::OK;
}

return_type ABBSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
#ifdef TRACING_ENABLED
  tracepoint(hardware_interface, write_start, info_.name.c_str());
#endif

  if (!is_activated_)
  {
    RCLCPP_WARN_THROTTLE(LOGGER, clock_, 1000, "Not activated. Skipping write");
    return return_type::OK;
  }

  // Ensure that there is not a too big difference between the current and the desired joint values
  for (auto& joint : motion_data_.groups[0].units[0].joints)
  {
    if (std::abs(joint.command.position - joint.state.position) > 0.05)
    {
      RCLCPP_WARN_THROTTLE(LOGGER, clock_, 1000, "Too big diff in joint %s. Skipping write", joint.name.c_str());
      // We are not returning ERROR because that would trigger a cleanup and restart and we don't want that
      return return_type::OK;
    }
  }

  RCLCPP_DEBUG_THROTTLE(LOGGER, clock_, 1000, "Writing to robot");
  egm_manager_->write(motion_data_);

#ifdef TRACING_ENABLED
  tracepoint(hardware_interface, write_end, info_.name.c_str());
#endif
  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)

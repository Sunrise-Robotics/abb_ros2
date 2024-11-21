/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * Modifications Copyright (c) 2022, PickNik Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

// This file is a modified copy from
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/src/initialization.cpp
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/src/verification.cpp

#include <abb_hardware_interface/utilities.hpp>
#include <stdexcept>

// New includes
#include <Poco/Net/HTTPSClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/URI.h>
#include <Poco/DOM/DOMParser.h>
#include <Poco/DOM/Document.h>
#include <Poco/DOM/NodeList.h>
#include <Poco/DOM/Element.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <Poco/Net/AcceptCertificateHandler.h>
#include <Poco/Net/SSLManager.h>
#include <Poco/Net/Context.h>
#include <Poco/Base64Encoder.h>
// End of new includes

#include <rclcpp/rclcpp.hpp>

namespace abb
{
namespace robot
{
namespace utilities
{
namespace
{
/**
 * \brief Max number of attempts when trying to connect to a robot controller via RWS.
 */
constexpr unsigned int RWS_MAX_CONNECTION_ATTEMPTS{ 5 };

/**
 * \brief Error message for failed connection attempts when trying to connect to a robot controller via RWS.
 */
constexpr char RWS_CONNECTION_ERROR_MESSAGE[]{ "Failed to establish RWS connection to the robot controller" };

/**
 * \brief Time [s] to wait before trying to reconnect to a robot controller via RWS.
 */
constexpr uint8_t RWS_RECONNECTION_WAIT_TIME{ 1 };
auto LOGGER = rclcpp::get_logger("ABBHardwareInterfaceUtilities");
}  // namespace

RobotControllerDescription establishRWSConnection(RWSManager& rws_manager, const std::string& robot_controller_id,
                                                  const bool no_connection_timeout)
{
  unsigned int attempt{ 0 };

  while (rclcpp::ok() && (no_connection_timeout || attempt++ < RWS_MAX_CONNECTION_ATTEMPTS))
  {
    try
    {
      return rws_manager.collectAndParseSystemData(robot_controller_id);
    }
    catch (const std::runtime_error& exception)
    {
      if (!no_connection_timeout)
      {
        RCLCPP_WARN_STREAM(LOGGER, RWS_CONNECTION_ERROR_MESSAGE << " (attempt " << attempt << "/"
                                                                << RWS_MAX_CONNECTION_ATTEMPTS << "), reason: '"
                                                                << exception.what() << "'");
      }
      else
      {
        RCLCPP_WARN_STREAM(LOGGER, RWS_CONNECTION_ERROR_MESSAGE << " (waiting indefinitely), reason: '"
                                                                << exception.what() << "'");
      }
      rclcpp::sleep_for(std::chrono::seconds(RWS_RECONNECTION_WAIT_TIME));
    }
  }

  throw std::runtime_error{ RWS_CONNECTION_ERROR_MESSAGE };
}

void verifyRobotWareVersion(const RobotWareVersion& rw_version)
{
  if (rw_version.major_number() == 6 && rw_version.minor_number() < 7 && rw_version.patch_number() < 1)
  {
    auto error_message{ "Unsupported RobotWare version (" + rw_version.name() + ", need at least 6.07.01)" };

    RCLCPP_FATAL_STREAM(LOGGER, error_message);
    throw std::runtime_error{ error_message };
  }
}

bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators)
{
  return system_indicators.addins().state_machine_1_0() || system_indicators.addins().state_machine_1_1();
}

// This function works as "stand-alone" does not make any use of the RWS managers. That is because
// the RWS managers (in their current state) are not compatible with the Omnicore controllers. The
// primary use of this function is to allow the hardware interface to retrieve the current joint
// values from the controller before EGM is initialized. This prevents sudden jumps in the joint
// states which would otherwise occur when starting the hardware interface.
std::vector<abb::robot::InitialJointValue> getRWSJointsFromController(
    const std::string& ip, const int port)
{
    std::vector<abb::robot::InitialJointValue> initial_joints;

    try {
        // Set up SSL context to accept all certificates
        Poco::Net::Context::Ptr context = new Poco::Net::Context(
            Poco::Net::Context::CLIENT_USE, "", "", "",
            Poco::Net::Context::VERIFY_NONE, 9, false,
            "ALL:!ADH:!LOW:!EXP:!MD5:@STRENGTH");

        // Create a certificate handler that accepts all certificates
        Poco::SharedPtr<Poco::Net::InvalidCertificateHandler> ptrHandler =
            new Poco::Net::AcceptCertificateHandler(false);

        // Install the custom certificate handler
        Poco::Net::SSLManager::instance().initializeClient(nullptr, ptrHandler, context);

        // Construct the URL and create session
        Poco::URI uri("https://" + ip + ":" + std::to_string(port) +
                     "/rw/motionsystem/mechunits/ROB_1/jointtarget");

        // Create session with the custom context
        Poco::Net::HTTPSClientSession session(uri.getHost(), uri.getPort(), context);
        session.setKeepAlive(true);

        // Create and send request with authentication
        Poco::Net::HTTPRequest request(Poco::Net::HTTPRequest::HTTP_GET, uri.getPathAndQuery());

        // Add Accept header
        request.set("Accept", "application/xhtml+xml;v=2.0");

        // Add Basic Authentication
        std::string auth = "Default User:robotics";
        std::ostringstream encodedAuth;
        Poco::Base64Encoder encoder(encodedAuth);
        encoder << auth;
        encoder.close();
        request.setCredentials("Basic", encodedAuth.str());

        session.sendRequest(request);

        // Get response
        Poco::Net::HTTPResponse response;
        std::istream& rs = session.receiveResponse(response);

        if (response.getStatus() != Poco::Net::HTTPResponse::HTTP_OK) {
            throw std::runtime_error("HTTP request failed with status: " +
                std::to_string(response.getStatus()));
        }

        // Read the response
        std::stringstream ss;
        ss << rs.rdbuf();
        std::string response_string = ss.str();

        // Parse XML response using Poco::XML
        Poco::XML::DOMParser parser;
        Poco::AutoPtr<Poco::XML::Document> doc = parser.parseString(response_string);

        // Navigate to the li element with class "ms-jointtarget"
        Poco::XML::Element* root = doc->documentElement();
        if (!root) {
            throw std::runtime_error("Failed to find root element in XML.");
        }

        // Find the li element with class "ms-jointtarget"
        Poco::XML::Element* body = root->getChildElement("body");
        if (!body) {
            throw std::runtime_error("Failed to find body element.");
        }

        Poco::XML::Element* div = body->getChildElement("div");
        if (!div) {
            throw std::runtime_error("Failed to find div element.");
        }

        Poco::XML::Element* ul = div->getChildElement("ul");
        if (!ul) {
            throw std::runtime_error("Failed to find ul element.");
        }

        Poco::XML::Element* li = ul->getChildElement("li");
        if (!li) {
            throw std::runtime_error("Failed to find li element.");
        }

        // Extract joint values
        for (int i = 1; i <= 6; ++i) {
            std::string joint_name = "rax_" + std::to_string(i);

            // Get all span elements
            Poco::XML::NodeList* spans = li->getElementsByTagName("span");
            Poco::XML::Element* joint_elem = nullptr;

            // Find the span with the matching class
            for (unsigned long i = 0; i < spans->length(); ++i) {
                Poco::XML::Element* span = static_cast<Poco::XML::Element*>(spans->item(i));
                if (span->getAttribute("class") == joint_name) {
                    joint_elem = span;
                    break;
                }
            }

            if (!joint_elem) {
                throw std::runtime_error("Failed to find " + joint_name + " in XML.");
            }

            InitialJointValue initial_value;
            initial_value.position_state = std::stod(joint_elem->innerText()) * M_PI / 180.0;
            initial_value.velocity_state = 0;
            initial_value.position_command = std::stod(joint_elem->innerText()) * M_PI / 180.0;
            initial_value.velocity_command = 0;
            initial_joints.push_back(initial_value);

            // Release the NodeList
            spans->release();
        }
    }
    catch (const Poco::Exception& e) {
        throw std::runtime_error("Poco error: " + std::string(e.what()));
    }

    return initial_joints;
}

}  // namespace utilities
}  // namespace robot
}  // namespace abb

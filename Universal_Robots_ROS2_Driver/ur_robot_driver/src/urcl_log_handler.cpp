// Copyright 2021 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.

#include <memory>
#include <utility>

#include "ur_robot_driver/urcl_log_handler.hpp"
#include "rclcpp/logging.hpp"

namespace ur_robot_driver
{
bool g_registered = false;
std::unique_ptr<UrclLogHandler> g_log_handler(new UrclLogHandler);

UrclLogHandler::UrclLogHandler() = default;

void UrclLogHandler::log(const char* file, int line, urcl::LogLevel loglevel, const char* message)
{
  rcutils_log_location_t location = { "", file, static_cast<size_t>(line) };
  switch (loglevel) {
    case urcl::LogLevel::DEBUG:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, "UR_Client_Library", "%s", message);
      break;
    case urcl::LogLevel::INFO:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, "UR_Client_Library", "%s", message);
      break;
    case urcl::LogLevel::WARN:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, "UR_Client_Library", "%s", message);
      break;
    case urcl::LogLevel::ERROR:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, "UR_Client_Library", "%s", message);
      break;
    case urcl::LogLevel::FATAL:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, "UR_Client_Library", "%s", message);
      break;
    default:
      break;
  }
}

void registerUrclLogHandler()
{
  if (g_registered == false) {
    // Log level is decided by ROS2 log level
    urcl::setLogLevel(urcl::LogLevel::DEBUG);
    urcl::registerLogHandler(std::move(g_log_handler));
    g_registered = true;
  }
}

void unregisterUrclLogHandler()
{
  if (g_registered == true) {
    urcl::unregisterLogHandler();
    g_registered = false;
  }
}

}  // namespace ur_robot_driver

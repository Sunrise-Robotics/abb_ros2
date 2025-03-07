cmake_minimum_required(VERSION 3.5)
project(abb_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable tracing in the hardware interface
option(ENABLE_TRACING "Enable tracing in the hardware interface" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith
      -Winit-self -Wredundant-decls
      -Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    abb_egm_rws_managers
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

# Add Poco and TinyXML2 to dependencies
# Find Poco components manually
find_path(POCO_INCLUDE_DIRS Poco/Poco.h)
find_library(POCO_FOUNDATION_LIB PocoFoundation)
find_library(POCO_NET_LIB PocoNet)
find_library(POCO_NETSSL_LIB PocoNetSSL)
find_library(POCO_XML_LIB PocoXML)
find_library(POCO_UTIL_LIB PocoUtil)
find_library(POCO_JSON_LIB PocoJSON)

# Check if all libraries were found
if(NOT POCO_FOUNDATION_LIB OR NOT POCO_NET_LIB OR NOT POCO_NETSSL_LIB)
  message(FATAL_ERROR "Required Poco libraries not found")
endif()

# Optional libraries - don't fail if not found
if(NOT POCO_XML_LIB)
  message(WARNING "Poco XML library not found, some functionality may be disabled")
endif()

if(NOT POCO_JSON_LIB)
  message(WARNING "Poco JSON library not found, some functionality may be disabled")
endif()

if(NOT POCO_UTIL_LIB)
  message(WARNING "Poco Util library not found, some functionality may be disabled")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#############
## Tracing ##
#############

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include for generated header file
)


###########
## Build ##
###########

# Define source files
set(SOURCE_FILES
  src/abb_hardware_interface.cpp
  src/utilities.cpp
)

# Conditionally add tracepoint source file when tracing is enabled
if(ENABLE_TRACING)
  list(APPEND SOURCE_FILES src/hardware_interface_tp.c)
  add_compile_definitions(TRACING_ENABLED)
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  ${SOURCE_FILES}
)

# Add Poco and TinyXML2 to target dependencies
target_link_libraries(${PROJECT_NAME}
  ${POCO_FOUNDATION_LIB}
  ${POCO_NET_LIB}
  ${POCO_NETSSL_LIB}
  ${POCO_XML_LIB}
  ${POCO_UTIL_LIB}
  ${POCO_JSON_LIB}
  lttng-ust  # Add the LTTng UST library
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${POCO_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface abb_hardware_interface.xml)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable ament_copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

cmake_minimum_required(VERSION 3.15)

project(
  RobotModelUpdate
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/RobotModelUpdate.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/RobotModelUpdate.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RobotModelUpdate.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/RobotModelUpdate.cpp)

set(plugin_HDR src/RobotModelUpdate.h src/Convex.h)

set(AUTOLOAD_RobotModelUpdate_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load ${PLUGIN_NAME} plugin")
# The add_plugin macro is provided by mc_rtc
add_plugin(RobotModelUpdate "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

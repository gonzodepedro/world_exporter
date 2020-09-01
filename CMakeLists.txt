cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(plugin_ign)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo3 REQUIRED)
add_library(plugin_ign SHARED export_plugin.cc)
set_property(TARGET plugin_ign PROPERTY CXX_STANDARD 17)
target_link_libraries(plugin_ign
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo3::ignition-gazebo3)

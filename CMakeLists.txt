cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(plugin_ign)

#--------------------------------------
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
find_package(ignition-gazebo4 REQUIRED)

#--------------------------------------
# Find ignition-rendering
find_package(ignition-rendering4 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering4_VERSION_MAJOR})


add_library(plugin_ign SHARED export_plugin.cc)
set_property(TARGET plugin_ign PROPERTY CXX_STANDARD 17)
target_link_libraries(plugin_ign
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-rendering4::ignition-rendering4
  PRIVATE ignition-gazebo4::ignition-gazebo4
)

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(ExportWorldToMesh)

set(CMAKE_AUTOMOC ON)

#--------------------------------------
find_package(ignition-gazebo4 REQUIRED COMPONENTS gui)

#--------------------------------------
# Find ignition-rendering
find_package(ignition-rendering4 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering4_VERSION_MAJOR})

QT5_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_NAME}.cc
  ${resources_RCC}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-gazebo4::gui
  PRIVATE ignition-rendering4::ignition-rendering4
  PRIVATE ignition-gazebo4::ignition-gazebo4
)

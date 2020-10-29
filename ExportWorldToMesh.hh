/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef IGNITION_GAZEBO_EXPORTWORLDTOMESH_HH_
#define IGNITION_GAZEBO_EXPORTWORLDTOMESH_HH_

#include <ignition/common/ColladaExporter.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

#include <ignition/math/Matrix4.hh>


/// \brief Example of a GUI plugin that has access to entities and components.
class ExportWorldToMesh : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: ExportWorldToMesh();

  /// \brief Destructor
  public: ~ExportWorldToMesh() override;

  /// \brief `ignition::gui::Plugin`s can overload this function to
  /// receive custom configuration from an XML file. Here, it comes from the
  /// SDF.
  ///
  /// <gui>
  ///   <plugin ...> <!-- this is the plugin element -->
  ///     ...
  ///   </plugin>
  /// </gui>
  ///
  /// \param[in] _pluginElem SDF <plugin> element. Will be null if the plugin
  /// is loaded without any XML configuration.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief GUI systems can overload this function to receive updated simulation
  /// state. This is called whenever the server sends state updates to the GUI.
  /// \param[in] _info Simulation information such as time.
  /// \param[in] _ecm Entity component manager, which can be used to get the
  /// latest information about entities and components.
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

    private:
      bool rendered{false};
      ignition::common::Mesh worldMesh;
      ignition::gazebo::Entity world;
      std::map<ignition::gazebo::Entity, std::pair<ignition::math::Pose3d, ignition::gazebo::Entity>> poses;
      std::vector<ignition::math::Matrix4d> subMeshMatrix;
      ignition::common::ColladaExporter exporter;

    private: ignition::math::Pose3d GetPose(ignition::gazebo::Entity parent_, ignition::math::Pose3d currentPose);

};

#endif

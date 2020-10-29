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

#include <ignition/common/Material.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Transparency.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/gui/GuiEvents.hh>

#include <ignition/plugin/Register.hh>

#include <sdf/Visual.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>

#include "ExportWorldToMesh.hh"

/////////////////////////////////////////////////
ExportWorldToMesh::ExportWorldToMesh() = default;

/////////////////////////////////////////////////
ExportWorldToMesh::~ExportWorldToMesh() = default;

/////////////////////////////////////////////////
void ExportWorldToMesh::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "World exporter plugin";

  std::cout << "RUNNING EXPORTER PLUGIN" << std::endl;
  this->rendered = false;

}


ignition::math::Pose3d ExportWorldToMesh::GetPose(ignition::gazebo::Entity parent_, ignition::math::Pose3d currentPose)
{
  std::stack<ignition::math::Pose3d> posesStack;
  std::pair<ignition::math::Pose3d, ignition::gazebo::Entity> curr;
  ignition::math::Pose3d localPose;

  posesStack.push(currentPose);
  while (this->world != parent_){
    curr = this->poses.at(parent_);
    igndbg << ">>  curr: " << curr.first << " - " << curr.second << std::endl;
    posesStack.push(curr.first);
    parent_ = curr.second;
  }
  while (!posesStack.empty())
  {
    localPose = localPose * posesStack.top();
    posesStack.pop();
  }
  igndbg << ">>>>  - POSE : " << localPose <<std::endl;

  return localPose;
}


//////////////////////////////////////////////////
void ExportWorldToMesh::Update(const ignition::gazebo::UpdateInfo & /*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{

  if(this->rendered) return;

  worldMesh.SetName("ExportedWorld");

  _ecm.Each<ignition::gazebo::components::World, ignition::gazebo::components::Name >(
      [&](const ignition::gazebo::Entity & _Entity,
        const ignition::gazebo::components::World *,
        const ignition::gazebo::components::Name *_name
      )->bool
      {
        igndbg << ">>>> World : " << _Entity << std::endl;
        this->world = _Entity;
        return true;
      });

  _ecm.Each<ignition::gazebo::components::Model, ignition::gazebo::components::Name, ignition::gazebo::components::Pose,
            ignition::gazebo::components::ParentEntity>(
      [&](const ignition::gazebo::Entity &_Entity,
          const ignition::gazebo::components::Model *,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::Pose *_pose,
          const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        igndbg << ">>>> Model : " << _Entity << std::endl;
        this->poses.insert(std::pair<ignition::gazebo::Entity,std::pair<ignition::math::Pose3d, ignition::gazebo::Entity>>(_Entity, std::pair<ignition::math::Pose3d, ignition::gazebo::Entity>( _pose->Data(), _parent->Data()) ));
        return true;
      });

  _ecm.Each<ignition::gazebo::components::Link, ignition::gazebo::components::Name, ignition::gazebo::components::Pose,
            ignition::gazebo::components::ParentEntity>(
      [&](const ignition::gazebo::Entity &_Entity,
          const ignition::gazebo::components::Link *,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::Pose *_pose,
          const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        this->poses.insert(std::pair<ignition::gazebo::Entity,std::pair<ignition::math::Pose3d, ignition::gazebo::Entity>>(_Entity, std::pair<ignition::math::Pose3d, ignition::gazebo::Entity>( _pose->Data(), _parent->Data()) ));
        return true;
      });


  _ecm.Each<ignition::gazebo::components::Visual,
            ignition::gazebo::components::Name,
            ignition::gazebo::components::Pose,
            ignition::gazebo::components::Geometry,
            ignition::gazebo::components::Transparency,
            ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_Entity,
        const ignition::gazebo::components::Visual *,
        const ignition::gazebo::components::Name *_name,
        const ignition::gazebo::components::Pose *_pose,
        const ignition::gazebo::components::Geometry *_geom,
        const ignition::gazebo::components::Transparency *_transparency,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
    {

      std::string name = _name->Data().empty() ? std::to_string(_Entity) :
          _name->Data();

      igndbg << "====================== Starting: " << name << " - " << _Entity << "  ====================== " << std::endl;

      ignition::math::Pose3d localPose = this->GetPose(_parent->Data(), _pose->Data());

      ignition::common::MaterialPtr mat(new ignition::common::Material());
      auto material = _ecm.Component<ignition::gazebo::components::Material>(_Entity);
      if (material != nullptr){
        mat->SetDiffuse(material->Data().Diffuse());
        mat->SetAmbient(material->Data().Ambient());
        mat->SetEmissive(material->Data().Emissive());
        mat->SetSpecular(material->Data().Specular());
        mat->SetTransparency(_transparency->Data());
      }

      const ignition::common::Mesh * mesh;
      std::weak_ptr<ignition::common::SubMesh> subm;
      ignition::math::Vector3d scale;
      ignition::math::Matrix4d matrix(localPose);
      ignition::common::MeshManager *meshManager =
          ignition::common::MeshManager::Instance();

      if (_geom->Data().Type() == sdf::GeometryType::BOX)
      {
        igndbg << "It is a Box!"  << std::endl;
        if(meshManager->HasMesh("unit_box"))
        {
          igndbg << "Box Found..."  << std::endl;
          mesh = meshManager->MeshByName("unit_box");
          scale=_geom->Data().BoxShape()->Size();
          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Box NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
      {
        if(meshManager->HasMesh("unit_cylinder"))
        {
          igndbg << "Cylinder Found"  << std::endl;
          mesh = meshManager->MeshByName("unit_cylinder");
          scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = _geom->Data().CylinderShape()->Length();

          igndbg << "Pose : x: " << localPose << std::endl;

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Cylinder NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
      {
        igndbg << "It is a Plane!"  << std::endl;
        if(meshManager->HasMesh("unit_plane"))
        {
          igndbg << "Plane Found"  << std::endl;
          // // Create a rotation for the plane mesh to account for the normal vector.
          mesh = meshManager->MeshByName("unit_plane");

          scale.X() = _geom->Data().PlaneShape()->Size().X();
          scale.Y() = _geom->Data().PlaneShape()->Size().Y();

          // // The rotation is the angle between the +z(0,0,1) vector and the
          // // normal, which are both expressed in the local (Visual) frame.
          ignition::math::Vector3d normal = _geom->Data().PlaneShape()->Normal();
          localPose.Rot().From2Axes(ignition::math::Vector3d::UnitZ, normal.Normalized());

          matrix = ignition::math::Matrix4d(localPose);

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Plane NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
      {
        igndbg << "It is a Sphere!"  << std::endl;
        if(meshManager->HasMesh("unit_sphere"))
        {
          igndbg << "Sphere Found"  << std::endl;
          mesh = meshManager->MeshByName("unit_sphere");

          scale.X() = _geom->Data().SphereShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = scale.X();

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Sphere NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::MESH)
      {
          igndbg << "It is a mesh!"  << std::endl;

          auto fullPath = ignition::gazebo::asFullPath(_geom->Data().MeshShape()->Uri(),
                                   _geom->Data().MeshShape()->FilePath());

          igndbg << "Uri : " << _geom->Data().MeshShape()->Uri() << std::endl;
          igndbg << "Path : " << _geom->Data().MeshShape()->FilePath() << std::endl;
          igndbg << "fullPath : " << fullPath << std::endl;

          if (fullPath.empty())
          {
            igndbg << "Mesh geometry missing uri" << std::endl;
            return true;
          }
          mesh = meshManager->Load(fullPath);

          igndbg << "Submesh count : " << mesh->SubMeshCount() << std::endl;

          if(!mesh) {
            igndbg << "MESH NOT FOUND!" << std::endl;
            return true;
          }

          for(int k=0; k < mesh->SubMeshCount(); k++)
          {

            igndbg << ">>>>>>>>> SUBMESH : " << k << std::endl;
            int j = mesh->SubMeshByIndex(k).lock()->MaterialIndex();

            igndbg << "Query Material index : " << j << std::endl;

            igndbg << "Material : " << mesh->MaterialByIndex(mesh->SubMeshByIndex(k).lock()->MaterialIndex()) << std::endl;
            igndbg << "Material Ptr: " << mesh->MaterialByIndex(mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get() << std::endl;

            igndbg << "Material index on worldmesh: " << worldMesh.IndexOfMaterial(mesh->MaterialByIndex(mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get()) << std::endl;

            int i = 0;
            if( j != -1 )
            {
              i = worldMesh.IndexOfMaterial(mesh->MaterialByIndex(mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get());
              if (i == -1)
              {
                i = worldMesh.AddMaterial(mesh->MaterialByIndex(mesh->SubMeshByIndex(k).lock()->MaterialIndex() ));
              }
            }
            else
            {
              i = worldMesh.AddMaterial(mat);
            }

            igndbg << "Inserted Material index : " << i << std::endl;

            igndbg << "Texture Image : " << worldMesh.MaterialByIndex(i)->TextureImage() << std::endl;

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(k).lock().get());
            subm.lock()->SetMaterialIndex(i);
            igndbg << "Scale : " <<_geom->Data().MeshShape()->Scale() << std::endl;

            subm.lock()->Scale(_geom->Data().MeshShape()->Scale());
            subMeshMatrix.push_back(matrix);

            igndbg << ">>>>>>>>>" << std::endl;
          }

      } else {
        igndbg << "It is NOT a mesh!"  << std::endl;
      }

      igndbg << "======================================================= " << std::endl;

      return true;
    });

    //TODO (@gonzo) unharcode export mesh name
    exporter.Export(&worldMesh, "./worldMesh", true, subMeshMatrix);
    this->rendered = true;
}




// Register this plugin
IGNITION_ADD_PLUGIN(ExportWorldToMesh,
                    ignition::gui::Plugin)

IGNITION_ADD_PLUGIN_ALIAS(ExportWorldToMesh, "WorldExporterPlugin")

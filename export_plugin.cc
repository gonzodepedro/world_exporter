#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Transparency.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/World.hh"


#include <ignition/plugin/Register.hh>
#include <sdf/Visual.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>

#include "ignition/gazebo/Util.hh"

#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include "ignition/rendering/Geometry.hh"
#include <ignition/rendering/Light.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include <ignition/math/Matrix4.hh>

#include <ignition/common/ColladaExporter.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class MyPlugin
      : public System,
        public ISystemPostUpdate
{

  public: MyPlugin()
  {

  }

  public: ~MyPlugin() override
  {

  }

  private:
    bool rendered{false};
    common::Mesh worldMesh;
    Entity world;
    std::map<Entity, std::pair<math::Pose3d, Entity>> poses;
    std::vector<math::Matrix4d> subMeshMatrix;

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {

    if(this->rendered) return;

    common::ColladaExporter exporter;
    worldMesh.SetName("ExportedWorld");

    _ecm.EachNew<components::World, components::Name >(
        [&](const Entity & _entity,
          const components::World *,
          const components::Name *_name
        )->bool
        {
          world = _entity;
          return true;
        });

    _ecm.Each<components::Model, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Model *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::ParentEntity *_parent)->bool
        {
          poses.insert(std::pair<Entity,std::pair<math::Pose3d, Entity>>(_entity, std::pair<math::Pose3d, Entity>( _pose->Data(), _parent->Data()) ));
          return true;
        });

    _ecm.Each<components::Link, components::Name, components::Pose,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Link *,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::ParentEntity *_parent)->bool
        {
          poses.insert(std::pair<Entity,std::pair<math::Pose3d, Entity>>(_entity, std::pair<math::Pose3d, Entity>( _pose->Data(), _parent->Data()) ));
          return true;
        });


    _ecm.Each<components::Visual,
              components::Name,
              components::Pose,
              components::Geometry,
              components::CastShadows,
              components::Transparency,
              components::ParentEntity>(
      [&](const ignition::gazebo::Entity &_entity,
          const components::Visual *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::Geometry *_geom,
          const components::CastShadows *_castShadows,
          const components::Transparency *_transparency,
          const components::ParentEntity *_parent)->bool
      {
        //_castShadows->Data();
        //_transparency->Data();

        // auto material = _ecm.Component<components::Material>(_entity);
        // if (material != nullptr)
        // {
        //   visual.SetMaterial(material->Data());
        // }

        std::string name = _name->Data().empty() ? std::to_string(_entity) :
            _name->Data();

        ignerr << "Starting: " << name << " - " << _entity <<std::endl;

        Entity parent = _parent->Data();
        std::stack<math::Pose3d> poses_stack;
        std::pair<math::Pose3d, Entity> curr;
        poses_stack.push(_pose->Data());
        while (world != parent){
          curr = poses.at(parent);
          poses_stack.push(curr.first);
          parent = curr.second;
        }
        math::Pose3d localPose = math::Pose3d::Zero;
        while (!poses_stack.empty())
        {
          localPose = localPose * poses_stack.top();
          poses_stack.pop();
        }
        ignerr << ">>>>  Name : " << name << " - POSE : " << localPose <<std::endl;

        const ignition::common::Mesh * mesh;
        std::weak_ptr<ignition::common::SubMesh> subm;
        math::Vector3d scale;
        math::Matrix4d matrix(localPose);
        ignition::common::MeshManager *meshManager =
            ignition::common::MeshManager::Instance();

        if (_geom->Data().Type() == sdf::GeometryType::BOX)
        {
          ignerr << "It is a Box!"  << std::endl;
          if(meshManager->HasMesh("unit_box"))
          {
            ignerr << "Box Found"  << std::endl;
            mesh = meshManager->MeshByName("unit_box");
            scale=_geom->Data().BoxShape()->Size();

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
            subm.lock()->Scale(scale);
            subMeshMatrix.push_back(matrix);
          }
          else
          {
            ignerr << "Box NOT Found"  << std::endl;
          }
        }
        else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
        {
          if(meshManager->HasMesh("unit_cylinder"))
          {
            ignerr << "Cylinder Found"  << std::endl;
            mesh = meshManager->MeshByName("unit_cylinder");
            scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
            scale.Y() = scale.X();
            scale.Z() = _geom->Data().CylinderShape()->Length();

            ignerr << "Pose : x: " << localPose << std::endl;

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
            subm.lock()->Scale(scale);
            subMeshMatrix.push_back(matrix);
          }
          else
          {
            ignerr << "Cylinder NOT Found"  << std::endl;
          }
        }
        else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
        {
          ignerr << "It is a Plane!"  << std::endl;
          if(meshManager->HasMesh("unit_plane"))
          {
            ignerr << "Plane Found"  << std::endl;
            mesh = meshManager->MeshByName("unit_plane");

            scale.X() = _geom->Data().PlaneShape()->Size().X();
            scale.Y() = _geom->Data().PlaneShape()->Size().Y();

            // // Create a rotation for the plane mesh to account for the normal vector.
            // // The rotation is the angle between the +z(0,0,1) vector and the
            // // normal, which are both expressed in the local (Visual) frame.
            math::Vector3d normal = _geom->Data().PlaneShape()->Normal();
            localPose.Rot().From2Axes(math::Vector3d::UnitZ, normal.Normalized());

            matrix = math::Matrix4d(localPose);

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
            subm.lock()->Scale(scale);
            subMeshMatrix.push_back(matrix);
          }
          else
          {
            ignerr << "Plane NOT Found"  << std::endl;
          }
        }
        else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
        {
          ignerr << "It is a Sphere!"  << std::endl;
          if(meshManager->HasMesh("unit_sphere"))
          {
            ignerr << "Sphere Found"  << std::endl;
            mesh = meshManager->MeshByName("unit_sphere");

            scale.X() = _geom->Data().SphereShape()->Radius() * 2;
            scale.Y() = scale.X();
            scale.Z() = scale.X();

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
            subm.lock()->Scale(scale);
            subMeshMatrix.push_back(matrix);
          }
          else
          {
            ignerr << "Sphere NOT Found"  << std::endl;
          }
        }
        else if (_geom->Data().Type() == sdf::GeometryType::MESH)
        {
            ignerr << "It is a mesh!"  << std::endl;

            auto fullPath = asFullPath(_geom->Data().MeshShape()->Uri(),
                                     _geom->Data().MeshShape()->FilePath());

            ignerr << "Uri : " << _geom->Data().MeshShape()->Uri() << std::endl;
            ignerr << "Path : " << _geom->Data().MeshShape()->FilePath() << std::endl;
            ignerr << "fullPath : " << fullPath << std::endl;

            if (fullPath.empty())
            {
              ignerr << "Mesh geometry missing uri" << std::endl;
              return true;
            }
            mesh = meshManager->Load(_geom->Data().MeshShape()->Uri());

            if(!mesh) {
              return true;
            }

            subm = worldMesh.AddSubMesh(*mesh->SubMeshByIndex(0).lock().get());
            subm.lock()->Scale(_geom->Data().MeshShape()->Scale());
            subMeshMatrix.push_back(matrix);

            ignerr << "--- MATRIX : " << matrix << std::endl;


        } else {
          ignerr << "It is NOT a mesh!"  << std::endl;
        }

        // rendering::MaterialPtr material{nullptr};
        // if (_visual.Material())
        // {
        //   material = this->LoadMaterial(*_visual.Material());
        // } else if (__geom->Data().Type() != sdf::GeometryType::MESH)
        // {
        //   MaterialPtr mat(new Material());
        // }


        return true;
      });

      exporter.Export(&worldMesh, "./worldMesh", true, subMeshMatrix);
      this->rendered = true;
  }
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    System,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

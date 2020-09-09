#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Transparency.hh"

#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/Material.hh"
#include <ignition/plugin/Register.hh>
#include <sdf/Visual.hh>
#include <sdf/Mesh.hh>

#include <ignition/common/MeshManager.hh>

#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Light.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>


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

  public: std::map<ignition::gazebo::Entity, rendering::VisualPtr> visuals;
  public: bool rendered{false};
  public: Entity worldId{0};
  public: rendering::ScenePtr scene;

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    if(this->rendered) return;
    //
    // std::map<std::string, std::string> params;
    // if (this->dataPtr->useCurrentGLContext)
    //   params["useCurrentGLContext"] = "1";
    // scene = std::move(rendering::engine("ogre2", params)->SceneByName(this->dataPtr->sceneName));
    //
    //
    // _ecm.Each<components::World, components::Scene>(
    //     [&](const Entity & _entity,
    //       const components::World *,
    //       const components::Scene *_scene)->bool
    //     {
    //       this->worldId = _entity;
    //       const sdf::Scene &sceneSdf = _scene->Data();
    //       // this->newScenes.push_back(sceneSdf);
    //       return true;
    //     });

    // ignmsg << "ExportPlugin::PostUpdate" << std::endl;
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
        sdf::Visual visual;
        visual.SetName(_name->Data());
        visual.SetRawPose(_pose->Data());
        visual.SetGeom(_geom->Data());
        visual.SetCastShadows(_castShadows->Data());
        visual.SetTransparency(_transparency->Data());

        ignerr << "Starting: " << _name->Data() << std::endl;

        if (!visual.Geom()){
          ignerr << "No Geom"  << std::endl;
          return true;
        }

//----------- Get Mesh

        if (visual.Geom()->Type() == sdf::GeometryType::MESH)
        {
            ignerr << "It is a mesh!"  << std::endl;

            // auto fullPath = asFullPath(_geom.MeshShape()->Uri(),
            //                         _geom.MeshShape()->FilePath());

            ignerr << "Uri : " << visual.Geom()->MeshShape()->Uri() << std::endl;
            ignerr << "Path : " << visual.Geom()->MeshShape()->FilePath() << std::endl;

            ignition::common::MeshManager *meshManager =
                ignition::common::MeshManager::Instance();
            const common::Mesh* mesh = meshManager->Load(visual.Geom()->MeshShape()->Uri());



            // if (fullPath.empty())
            // {
            //   ignerr << "Mesh geometry missing uri" << std::endl;
            //   return geom;
            // }
            // rendering::MeshDescriptor descriptor;
            //
            // // Assume absolute path to mesh file
            // descriptor.meshName = fullPath;
            // descriptor.subMeshName = _geom.MeshShape()->Submesh();
            // descriptor.centerSubMesh = _geom.MeshShape()->CenterSubmesh();
            //
            // ignition::common::MeshManager *meshManager =
            //   ignition::common::MeshManager::Instance();
            // descriptor.mesh = meshManager->Load(descriptor.meshName);
            // geom = this->dataPtr->scene->CreateMesh(descriptor);
            // scale = _geom.MeshShape()->Scale();
        } else {
          ignerr << "It is NOT a mesh!"  << std::endl;
        }

//-------------------------

        return true;
      });

      this->rendered = true;
  }
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    System,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

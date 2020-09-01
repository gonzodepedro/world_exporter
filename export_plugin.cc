#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Geometry.hh>
#include <sdf/Mesh.hh>
#include <ignition/common/MeshManager.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace rendering;

class MyPlugin
      : public System,
        public ISystemPostUpdate
{

  public: MyPlugin(): dataPtr(std::make_unique<MyPlugin>())
  {

  }

  public: ~MyPlugin() override
  {

  }

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    // ignmsg << "ExportPlugin::PostUpdate" << std::endl;
    _ecm.Each<components::Visual, components::Name>(
      [&](const ignition::gazebo::Entity &_entity, const components::Visual *_visual,
          const components::Name *_nameComp) -> bool
      {
        auto geo = *_visual.Geom();
        rendering::MeshDescriptor descriptor;
        auto fullPath = asFullPath(_geom.MeshShape()->Uri(),
        geo.MeshShape()->FilePath());
        descriptor.meshName = fullPath;
        descriptor.subMeshName = geo.MeshShape()->Submesh();
        descriptor.centerSubMesh = geo.MeshShape()->CenterSubmesh();

        ignition::common::MeshManager *meshManager =
          ignition::common::MeshManager::Instance();
        descriptor.mesh = meshManager->Load(descriptor.meshName);
        // rendering::GeometryPtr geom = his->dataPtr->scene->CreateMesh(descriptor); HERE WE ARE MISSING THE rendering::Scene.CreateMesh() method
        auto mesh = std::dynamic_pointer_cast<rendering::Mesh>(geom);
        std::cout << _entity << std::endl;
        return true;
      });
  }
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    System,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

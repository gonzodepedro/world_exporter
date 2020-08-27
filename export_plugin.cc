#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include <ignition/plugin/Register.hh>

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

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    // ignmsg << "ExportPlugin::PostUpdate" << std::endl;
    _ecm.Each<components::Visual, components::Name>(
      [&](const ignition::gazebo::Entity &_entity, const components::Visual *,
          const components::Name *_nameComp) -> bool
      {
        std::cout << _entity << std::endl;
        return true;
      });
  }
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    System,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

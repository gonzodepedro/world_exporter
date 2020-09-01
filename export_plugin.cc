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

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
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
//        visual.SetVisibilityFlags(_visibilityFlags->Data());

        // Optional components
        auto material = _ecm.Component<components::Material>(_entity);
        if (material != nullptr)
        {
          visual.SetMaterial(material->Data());
        }


        std::cout << _entity << std::endl << std::endl;

        if (this->visuals.find(_id) != this->visuals.end())
        {
          ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
                 << std::endl;
          //return rendering::VisualPtr();
          return true;
        }

        Entity _parentId = _parent->Data();
        rendering::VisualPtr parent;
        if (_parentId != this->dataPtr->worldId)
        {
          auto it = this->dataPtr->visuals.find(_parentId);
          if (it == this->dataPtr->visuals.end())
          {
            // It is possible to get here if the model entity is created then
            // removed in between render updates.
            return rendering::VisualPtr();
          }
          parent = it->second;
        }

        if (!_visual.Geom()) return true;

        return true;
      });

  }
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    System,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

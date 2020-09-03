#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include <ignition/plugin/Register.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/MeshDescriptor.hh>
#include "ignition/gazebo/Util.hh"
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace rendering;

class MyPlugin : public ignition::gazebo::GuiSystem
{

  Q_OBJECT

    /// \brief Custom property. Use this to create properties that can be read
    /// from the QML file. See the declarations below.
    Q_PROPERTY(
      QString customProperty
      READ CustomProperty
      WRITE SetCustomProperty
      NOTIFY CustomPropertyChanged
    )

  public: MyPlugin()
  {

  }

  public: ~MyPlugin() override
  {

  }

  public: void Update(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    // ignmsg << "ExportPlugin::PostUpdate" << std::endl;
    _ecm.Each<components::Visual, components::Name, components::Geometry>(
      [&](const ignition::gazebo::Entity &_entity, const components::Visual *,
          const components::Name *_nameComp,
          const components::Geometry *_geoComp) -> bool
      {
        // auto geo = _geoComp->Data();
        // // rendering::MeshDescriptor descriptor;
        // auto fullPath = asFullPath(geo.MeshShape()->Uri(),
        //   geo.MeshShape()->FilePath());
        // // descriptor.meshName = fullPath;
        // // descriptor.subMeshName = geo.MeshShape()->Submesh();
        // // descriptor.centerSubMesh = geo.MeshShape()->CenterSubmesh();
        //
        // ignition::common::MeshManager *meshManager =
        //   ignition::common::MeshManager::Instance();
        // // descriptor.mesh = meshManager->Load(descriptor.meshName);
        // auto mesh = meshManager->MeshByName(fullPath);
        // // rendering::GeometryPtr geom = his->dataPtr->scene->CreateMesh(descriptor); HERE WE ARE MISSING THE rendering::Scene.CreateMesh() method
        // // auto mesh = std::dynamic_pointer_cast<rendering::Mesh>(geom);
        //
        // if (mesh != nullptr)
        // {
        //   std::cout << "FOUND MESH AT " << fullPath << std::endl;
        // }
        this->SetCustomProperty(QString::fromStdString(_nameComp->Data()));
        return true;
      });
  }

  /// \brief Get the custom property as a string.
  /// \return Custom property
  public: Q_INVOKABLE QString CustomProperty() const
  {
  return this->customProperty;
  }

  /// \brief Set the custom property from a string.
  /// \param[in] _customProperty Custom property
  public: Q_INVOKABLE void SetCustomProperty(const QString &_customProperty)
  {
  this->customProperty = _customProperty;
  this->CustomPropertyChanged();
  }

  /// \brief Notify that custom property has changed
  signals: void CustomPropertyChanged();

  /// \brief Custom property
  private: QString customProperty;

};

IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gui::Plugin)

// IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"MyPlugin")

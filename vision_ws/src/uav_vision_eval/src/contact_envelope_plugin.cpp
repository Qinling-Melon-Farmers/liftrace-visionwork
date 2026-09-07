#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo {

// The conservative contact envelope is a scoring proxy, not an optical
// surface. Keep its mesh/height and physical contacts, but exclude ray sensors.
class ContactEnvelopePlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr config) override {
    if (!config->HasElement("link_name") || !config->HasElement("collision_name"))
      gzthrow("Contact envelope requires link_name and collision_name");
    const auto link = model->GetLink(config->Get<std::string>("link_name"));
    if (!link) gzthrow("Contact envelope link is missing");
    const auto collision = link->GetCollision(config->Get<std::string>("collision_name"));
    if (!collision) gzthrow("Contact envelope collision is missing");

    // Gazebo's ray space has category GZ_SENSOR_COLLIDE and excludes that
    // category. Both masks are needed: world/body collisions still match.
    collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
    collision->SetCollideBits(GZ_ALL_COLLIDE & ~GZ_SENSOR_COLLIDE);
    gzmsg << "Contact-only envelope configured: " << collision->GetScopedName()
          << "; ray visibility disabled, physical contacts preserved\n";
  }
};

GZ_REGISTER_MODEL_PLUGIN(ContactEnvelopePlugin)
}  // namespace gazebo

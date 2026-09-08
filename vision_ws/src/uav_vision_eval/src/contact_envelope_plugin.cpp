#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactsState.h>
#include <ros/ros.h>
#include <map>
#include <mutex>

namespace gazebo {

// The conservative contact envelope is a scoring proxy, not an optical
// surface. Keep its mesh/height and physical contacts, but exclude ray sensors.
class ContactEnvelopePlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr config) override {
    if (!config->HasElement("link_name") || !config->HasElement("collision_name"))
      gzthrow("Contact envelope requires link_name and collision_name");
    const auto link = FindLink(model, config->Get<std::string>("link_name"));
    if (!link) gzthrow("Contact envelope link is missing");
    const auto collision = link->GetCollision(config->Get<std::string>("collision_name"));
    if (!collision) gzthrow("Contact envelope collision is missing");

    // Gazebo's ray space has category GZ_SENSOR_COLLIDE and excludes that
    // category. Both masks are needed: world/body collisions still match.
    collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
    collision->SetCollideBits(GZ_ALL_COLLIDE & ~GZ_SENSOR_COLLIDE);
    // A closed CAD housing is not the lidar's optical window. The installed
    // emission origin is internal; retain physical housing contacts while
    // excluding this explicitly named self-housing from simulated rays.
    if (config->HasElement("ray_transparent_link")) {
      const auto housing_link = FindLink(model, config->Get<std::string>("ray_transparent_link"));
      if (!housing_link || !config->HasElement("ray_transparent_collision"))
        gzthrow("Ray-transparent housing configuration is incomplete");
      const auto housing = housing_link->GetCollision(config->Get<std::string>("ray_transparent_collision"));
      if (!housing) gzthrow("Ray-transparent housing collision is missing");
      housing->SetCategoryBits(GZ_SENSOR_COLLIDE);
      housing->SetCollideBits(GZ_ALL_COLLIDE & ~GZ_SENSOR_COLLIDE);
    }
    gzmsg << "Contact-only envelope configured: " << collision->GetScopedName()
          << "; ray visibility disabled, physical contacts preserved\n";

    // Optional complete-airframe observation. A guard-only bumper misses
    // physical contacts of included iris rotors and the sensor housing.
    // This does not alter any of their collision masks or dimensions.
    if (config->HasElement("contacts_topic")) {
      if (!ros::isInitialized()) gzthrow("Gazebo ROS must initialize contact recording");
      world_ = model->GetWorld();
      filter_name_ = model->GetName() + "_all_airframe_contacts";
      std::map<std::string, physics::CollisionPtr> collisions;
      CollectCollisions(model, collisions);
      const auto topic = world_->Physics()->GetContactManager()->CreateFilter(
          filter_name_, collisions);
      node_.reset(new transport::Node());
      node_->Init(world_->Name());
      ros_node_.reset(new ros::NodeHandle());
      publisher_ = ros_node_->advertise<gazebo_msgs::ContactsState>(
          config->Get<std::string>("contacts_topic"), 20);
      subscriber_ = node_->Subscribe(topic, &ContactEnvelopePlugin::OnContacts, this);
      update_ = event::Events::ConnectWorldUpdateEnd(
          std::bind(&ContactEnvelopePlugin::OnUpdate, this));
      gzmsg << "Full airframe contact recording: " << collisions.size()
            << " collision shapes on " << config->Get<std::string>("contacts_topic") << "\n";
    }
  }

  ~ContactEnvelopePlugin() override {
    update_.reset();
    subscriber_.reset();
    if (world_ && !filter_name_.empty())
      world_->Physics()->GetContactManager()->RemoveFilter(filter_name_);
  }

 private:
  static physics::LinkPtr FindLink(const physics::ModelPtr &model,
      const std::string &name) {
    for (const auto &link : model->GetLinks())
      if (link->GetName() == name || link->GetScopedName() == name) return link;
    for (const auto &nested : model->NestedModels()) {
      const auto link = FindLink(nested, name);
      if (link) return link;
    }
    return physics::LinkPtr();
  }

  static void CollectCollisions(const physics::ModelPtr &model,
      std::map<std::string, physics::CollisionPtr> &result) {
    for (const auto &link : model->GetLinks())
      for (const auto &collision : link->GetCollisions())
        result[collision->GetScopedName()] = collision;
    for (const auto &nested : model->NestedModels()) CollectCollisions(nested, result);
  }

  void OnContacts(ConstContactsPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &contact : message->contact()) {
      if (pending_.empty())
        first_contact_stamp_ = ros::Time(contact.time().sec(), contact.time().nsec());
      gazebo_msgs::ContactState state;
      state.collision1_name = contact.collision1();
      state.collision2_name = contact.collision2();
      state.depths.assign(contact.depth().begin(), contact.depth().end());
      pending_[std::make_pair(state.collision1_name, state.collision2_name)] = state;
    }
  }

  void OnUpdate() {
    const auto now = world_->SimTime();
    if ((now - last_publish_).Double() < 0.02) return;
    last_publish_ = now;
    gazebo_msgs::ContactsState out;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      out.header.stamp = pending_.empty() ? ros::Time(now.sec, now.nsec) : first_contact_stamp_;
      for (const auto &item : pending_) out.states.push_back(item.second);
      pending_.clear();
    }
    // Keep every contact episode, including impulses shorter than 20 ms;
    // publish an empty heartbeat when no collision was observed.
    publisher_.publish(out);
  }

  physics::WorldPtr world_;
  transport::NodePtr node_;
  transport::SubscriberPtr subscriber_;
  event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher publisher_;
  std::string filter_name_;
  common::Time last_publish_;
  ros::Time first_contact_stamp_;
  std::mutex mutex_;
  std::map<std::pair<std::string, std::string>, gazebo_msgs::ContactState> pending_;
};

GZ_REGISTER_MODEL_PLUGIN(ContactEnvelopePlugin)
}  // namespace gazebo

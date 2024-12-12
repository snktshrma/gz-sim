#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <iostream>

using namespace gz;
using namespace sim;

namespace proximity_plugin {
  class ProximityPlugin : public System,
        public ISystemConfigure,
        public ISystemPostUpdate {
   public:
    // Configure method called during initialization
    void Configure(const Entity &entity,
                   const std::shared_ptr<const sdf::Element> &,
                   EntityComponentManager &ecm,
                   EventManager &) override {
      // Store the model entity
      this->modelEntity = entity;

      // Retrieve the model name
      auto nameComp = ecm.Component<components::Name>(entity);
      if (!nameComp) {
        std::cerr << "Entity does not have a Name component." << std::endl;
        return;
      }

      // Store the model name
      this->modelName = nameComp->Data();

      // Ensure the entity has a Pose component
      if (!ecm.Component<components::Pose>(entity)) {
        std::cerr << "Entity does not have a Pose component." << std::endl;
        return;
      }

      std::cout << "ProximityPlugin loaded for model: " << this->modelName
                << std::endl;

      // Initialize the transport node
      this->node = std::make_unique<transport::Node>();

      // Create a publisher on a dynamic Gazebo topic
      std::string topic = "/" + this->modelName + "/pose";
      this->publisher = this->node->Advertise<msgs::Pose>(topic);
      if (!this->publisher) {
        std::cerr << "Failed to create publisher on topic: " << topic << std::endl;
      } else {
        std::cout << "Publishing pose data on topic: " << topic << std::endl;
      }
    }

    // PostUpdate is called every simulation step
    void PostUpdate(const UpdateInfo &,
                    const EntityComponentManager &ecm) override {
      // Get the pose component of the model
      auto poseComp = ecm.Component<components::Pose>(this->modelEntity);

      if (poseComp) {
        // Extract the pose
        const auto pose = poseComp->Data();

        // Print the position
        std::cout << "Position: [" << pose.Pos().X() << ", " << pose.Pos().Y()
                  << ", " << pose.Pos().Z() << "]" << std::endl;

        // Create a pose message
        msgs::Pose poseMsg;
        poseMsg.mutable_position()->set_x(pose.Pos().X());
        poseMsg.mutable_position()->set_y(pose.Pos().Y());
        poseMsg.mutable_position()->set_z(pose.Pos().Z());
        poseMsg.mutable_orientation()->set_w(pose.Rot().W());
        poseMsg.mutable_orientation()->set_x(pose.Rot().X());
        poseMsg.mutable_orientation()->set_y(pose.Rot().Y());
        poseMsg.mutable_orientation()->set_z(pose.Rot().Z());

        // Publish the message
        this->publisher.Publish(poseMsg);
      } else {
        std::cerr << "Pose component not found!" << std::endl;
      }
    }

   private:
    // Entity representing the model
    Entity modelEntity;

    // Model name
    std::string modelName;

    // Transport node
    std::unique_ptr<transport::Node> node;

    // Publisher for the model's pose
    transport::Node::Publisher publisher;
  };
}

// Register the plugin with Gazebo
GZ_ADD_PLUGIN(proximity_plugin::ProximityPlugin,
    System,
    proximity_plugin::ProximityPlugin::ISystemConfigure,
    proximity_plugin::ProximityPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(proximity_plugin::ProximityPlugin, "gz::sim::systems::ProximityPlugin")

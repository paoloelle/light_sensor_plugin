#ifndef LIGHT_SENSOR_HH_
#define LIGHT_SENSOR_HH_

#include <ignition/gazebo6/ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace custom_light_sensor
{

    class LightSensor:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate

    {
        /// \brief Constructor
        public: LightSensor();

        /// \brief Deconstructor
        public: ~LightSensor() override;

        // Documentation inherited
        public: void Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &_eventMgr) override;

        // Documentation inherited
        public : void PostUpdate(
                                const ignition::gazebo::UpdateInfo &_info,
                                const ignition::gazebo::EntityComponentManager &_ecm) override;

        /// \brief Maximum value
        public: double maxValue{1.0};

        /// \brief Minimum value
        public: double minValue{0.0};

        /// \brief Sensor update rate
        public: double updateRate{1.0};

        /// \brief Entity ID of the sensor
        public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};

        /// \brief Entity ID of the parent model
        public: ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};

        /// \brief Ignition tranport node
        public: ignition::transport::Node node;

        /// \brief Ignition transport publisher for publishing sensor data
        public: ignition::transport::Node::Publisher publisher;

        /// \brief Sim time when next update should occur
        public: std::chrono::steady_clock::duration nextUpdateTime
            {std::chrono::steady_clock::duration::zero()};


    };

}

#endif
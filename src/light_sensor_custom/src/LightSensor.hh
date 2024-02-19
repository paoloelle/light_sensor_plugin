#ifndef LIGHT_SENSOR_HH_
#define LIGHT_SENSOR_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>


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

        /// Documentation inherited
        public: void Configure(
                            const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr) override;

        /// Documentation inherited
        public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

        /// \brief store position of ligth sources
        public: std::vector<ignition::math::Pose3d> lightSourcesPoses;

        /// \brief update rate
        public: double updateRate{5.0};

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
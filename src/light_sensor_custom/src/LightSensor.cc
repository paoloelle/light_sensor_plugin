#include <ignition/gazebo/Light.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/math.hh>
#include <ignition/gazebo.hh>
#include <ignition/transport/TopicUtils.hh>
#include <ignition/msgs.hh>

#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/sensors/Noise.hh>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/LightType.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <sdf/Root.hh>

#include "LightSensor.hh"

using namespace custom_light_sensor;

LightSensor::LightSensor(){}

LightSensor::~LightSensor() = default;

void LightSensor::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
    {
		
		

		// questo metoodo sembra funzionare solo all'interno dei modelli
  		_ecm.Each<ignition::gazebo::components::Light,
				ignition::gazebo::components::Pose>(
				[&](const ignition::gazebo::Entity &_entity,
				const ignition::gazebo::components::Light *,
				const ignition::gazebo::components::Pose *_pose) -> bool
		{	

			
			ignition::math::Pose3d light_pose = _pose->Data();
			lightPoses.push_back(light_pose);


			return true;

		});

		std::cout << lightPoses <<std::endl;


		// set the topic for sensor data publication
		std::string topic = ignition::gazebo::scopedName(_entity, _ecm) + "/" + "light_value";
		topic = ignition::transport::TopicUtils::AsValidTopic(topic);

		// create the publisher
		this->publisher = this->node.Advertise<ignition::msgs::Float>(topic);


    }


    void LightSensor::PostUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        const ignition::gazebo::EntityComponentManager &_ecm)
        {
        	// Throttle the sensor updates using sim time
            // If update_rate is set to 0, it means unthrottled
            if (_info.simTime < this->nextUpdateTime && this->updateRate > 0)
                return;

            if (this->updateRate > 0.0){
                // Update the time the plugin should be loaded
                auto delta = std::chrono::duration_cast<std::chrono::milliseconds>
                (std::chrono::duration<double>(1.0 / this->updateRate));
                this->nextUpdateTime += delta;
            }

			// populate and publish the message
			double light_value = 0.0;

			/*for (const auto &pose : lightPoses){
				light_value+=pose;
			}*/

			//std:: cout << lightPoses.size() << std::endl;

			// time stamp the message with sim time
			ignition::msgs::Float msg;
			*msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_info.simTime);
			auto frame = msg.mutable_header()->add_data();

			// set frame id to scoped name of this sensor
			frame->set_key("frame_id");


			// populate sensor data
			msg.set_data(light_value);
			this->publisher.Publish(msg);

    	}

	// Register the plugin
	IGNITION_ADD_PLUGIN(custom_light_sensor::LightSensor,
        ignition::gazebo::System,
        LightSensor::ISystemConfigure,
        LightSensor::ISystemPostUpdate)
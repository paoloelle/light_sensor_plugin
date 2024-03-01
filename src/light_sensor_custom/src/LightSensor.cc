#include <ignition/gazebo/Light.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/math.hh>
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
#include <sdf/Light.hh>

#include "LightSensor.hh"

#include <ignition/plugin/Register.hh>


using namespace custom_light_sensor;

LightSensor::LightSensor(){}

LightSensor::~LightSensor() = default;

void LightSensor::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/
	)
	
    {	
		// get update rate from sdf file
		auto sdf = const_cast<sdf::Element *>(_sdf.get());
		this->updateRate = sdf->Get("update_rate", this->updateRate).first;

		// get and store the pose of each light element
		_ecm.Each<ignition::gazebo::components::Light,
		ignition::gazebo::components::Pose>(
		[&](const ignition::gazebo::Entity &_entity,
		const ignition::gazebo::components::Light *_light,
		const ignition::gazebo::components::Pose *_pose) -> bool
		{	
			
			ignition::math::Pose3d light_pose = _pose->Data();
			lightPoses.push_back(light_pose);

			return true;

		});


		// Get top level model this entity belongs to
		// this system is attached to a sensor model but we are only interested
		// in the top level vehicle model pose - this is later used for
		// computing distance to other models in the environment
		auto parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(
			_entity);
		this->entity = _entity;
		this->modelEntity = _entity;
		while (parent && _ecm.Component<ignition::gazebo::components::Model>(
				parent->Data()))
		{
			this->modelEntity = parent->Data();
			parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(
				parent->Data());
		}


		// set topic to publish sensor data to
		std::string topic = ignition::gazebo::scopedName(_entity, _ecm) + "/light_value";
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

			  // get the pose of the model
			const ignition::gazebo::components::Pose *poseComp =
				_ecm.Component<ignition::gazebo::components::Pose>(this->modelEntity);
			ignition::math::Pose3d entityPose = poseComp->Data();

			/* TODO for now the light detected will be only porportional to the sum of distances from source lights
			when I will find how to access to the attenuation factors I will change it (or I can hard code it) */
			
			double light_value = 0.0; // light detected

			for (auto & light_pose : lightPoses){

				light_value += entityPose.Pos().Distance(light_pose.Pos());

			}
			
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
IGNITION_ADD_PLUGIN(
	custom_light_sensor::LightSensor,
    ignition::gazebo::System,
    LightSensor::ISystemConfigure,
    LightSensor::ISystemPostUpdate)
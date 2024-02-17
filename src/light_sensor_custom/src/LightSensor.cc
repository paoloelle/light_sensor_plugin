#include <ignition/msgs.hh>
#include <ignition/plugin1/ignition/plugin/Register.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Pose.hh>


#include "LightSensor.hh"

using namespace custom_light_sensor;

LightSensor::LightSensor(){}

LightSensor::~LightSensor() = default;

void LightSensor::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
    {

        // parse configuration parameters from SDF file
        auto sdf = const_cast<sdf::Element *>(_sdf.get());
        this->updateRate = sdf->Get("update_rate", this->updateRate).first;

        // Get top level model this entity belongs to
        // this system is attached to a sensor model but we are only interested
        // in the top level vehicle model pose - this is later used for
        // computing distance to other models in the environment

        auto parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);
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
            std::string topic = ignition::gazebo::scopedName(_entity, _ecm) + "/light_sensor";
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

			// compute the inverse of the pose
  			// this is used later to convert pose of other entites into this model frame
  			auto inversePose = entityPose.Inverse();

			double light_value;

  			// Loop through all the models in simulation
  			_ecm.Each<ignition::gazebo::components::Model,
            	ignition::gazebo::components::Pose,
            	ignition::gazebo::components::ParentEntity>(
      			[&](const ignition::gazebo::Entity &_entity,
          		const ignition::gazebo::components::Model *,
          		const ignition::gazebo::components::Pose *_pose,
          		const ignition::gazebo::components::ParentEntity *_parent) -> bool
      		{
        		// skip self
				if (_entity == this->modelEntity)
				return true;

				// ignore nested models by checking to see if it has a parent entity
				// that is also a model
				const ignition::gazebo::components::ParentEntity *parentComp =
					_ecm.Component<ignition::gazebo::components::ParentEntity>(
					_parent->Data());
				if (parentComp &&
					_ecm.Component<ignition::gazebo::components::Model>(
					parentComp->Data()))
				return true;

				// get the model pose
				ignition::math::Pose3d pose = _pose->Data();

				// compute range
				double light_value = entityPose.Pos().Distance(pose.Pos());

				/*TODO
				qui vanno calcolati i valori delle letture del sensore*/


				return true;
			});

  		// populate and publish the message

  		// time stamp the message with sim time
  		ignition::msgs::Float msg;
  		*msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_info.simTime);
  		auto frame = msg.mutable_header()->add_data();

  		// set frame id to scoped name of this sensor
  		frame->set_key("frame_id");
  		std::string scopedName =
      		ignition::gazebo::removeParentScope(
      		ignition::gazebo::scopedName(this->entity, _ecm, "::", false), "::");
  		frame->add_value(scopedName);

  		// populate sensor data
  		msg.set_data(light_value);
  		this->publisher.Publish(msg);

        }

		// Register the plugin
		IGNITION_ADD_PLUGIN(custom_light_sensor::LightSensor,
                    ignition::gazebo::System,
                    LightSensor::ISystemConfigure,
                    LightSensor::ISystemPostUpdate)
    

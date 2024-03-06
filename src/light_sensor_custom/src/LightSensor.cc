

#include <math.h>

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "LightSensor.hh"

using namespace custom;

//////////////////////////////////////////////////
bool LightSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("light_sensor" != type)
  {
    gzerr << "Trying to load [light_sensor], but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:light_sensor"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("gz:light_sensor");

  if (!customElem->HasElement("noise"))
  {
    gzdbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    gzerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool LightSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  gz::msgs::Double msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  this->totalDistance = this->noise->Apply(this->totalDistance);

  msg.set_data(this->totalDistance);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void Odometer::NewPosition(const gz::math::Vector3d &_pos)
{
  if (!isnan(this->prevPos.X()))
  {
    this->totalDistance += this->prevPos.Distance(_pos);
  }
  this->prevPos = _pos;
}

//////////////////////////////////////////////////
const gz::math::Vector3d &Odometer::Position() const
{
  return this->prevPos;
}
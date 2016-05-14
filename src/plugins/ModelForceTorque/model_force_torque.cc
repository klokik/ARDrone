#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "model_force_torque.hh"


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelForceTorque);

/////////////////////////////////////////////////
ModelForceTorque::ModelForceTorque()
{
}

/////////////////////////////////////////////////
void ModelForceTorque::OnForceMsg(ConstVector3dPtr &_msg)
{
  this->lastForce = ignition::math::Vector3d(_msg->x(), _msg->y(), _msg->z());
}

/////////////////////////////////////////////////
void ModelForceTorque::OnTorqueMsg(ConstVector3dPtr &_msg)
{
  this->lastTorque = ignition::math::Vector3d(_msg->x(), _msg->y(), _msg->z());
}

/////////////////////////////////////////////////
void ModelForceTorque::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  // FIXME: set link name in SDF file
  this->link = this->model->GetLink("quadrotor");

  // Create the subscriber
  std::string topicName = std::string("~/") + _parent->GetName()
    + "/link/";
  this->forceSubscriber = node->Subscribe(
      topicName + "add_force", &ModelForceTorque::OnForceMsg, this);
  this->torqueSubscriber = node->Subscribe(
      topicName + "add_torque", &ModelForceTorque::OnTorqueMsg, this);
  gzmsg << "[model_location] Subscribed to receive forces in: "<< topicName+"add_[force,torque]"
        << std::endl;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ModelForceTorque::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ModelForceTorque::OnUpdate(const common::UpdateInfo &_info)
{
  this->link->AddForce(this->lastForce);
  this->link->AddTorque(this->lastTorque);
}
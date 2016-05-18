#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "model_location.hh"


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelLocation);

/////////////////////////////////////////////////
ModelLocation::ModelLocation()
{
}

/////////////////////////////////////////////////
void ModelLocation::OnPoseMsg(ConstPosePtr &_msg)
{
  // gzmsg << "[model_location] Received pose message" << std::endl;

  auto mpos = _msg->position();
  auto mrot = _msg->orientation();

  auto gpos = gazebo::math::Vector3(mpos.x(), mpos.y(), mpos.z());
  auto grot = gazebo::math::Quaternion(mrot.w(), mrot.x(), mrot.y(), mrot.z());

  if (_msg->has_id())
  {
    if (_msg->id() == 1)
      gpos = this->model->GetWorldPose().pos;
    if (_msg->id() == 2)
      grot = this->model->GetWorldPose().rot;
  }

  this->model->SetWorldPose({gpos, grot});
}

/////////////////////////////////////////////////
void ModelLocation::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  // Create the subscriber
  std::string poseTopicName = std::string("~/") + _parent->GetName()
    + "/pose/";
  this->poseSubscriber = node->Subscribe(
      poseTopicName + "modify", &ModelLocation::OnPoseMsg, this);
  gzmsg << "[model_location] Subscribed to receive poses in: "<< poseTopicName+"modify"
        << std::endl;

  // Create publisher
  this->posePublisher = node->Advertise<msgs::Pose>(poseTopicName + "info");
  gzmsg << "[model_location] Posting object poses to: "<< poseTopicName+"info"
        << std::endl;

  this->lastUpdateTime = common::Time(0.0);
  this->updatePeriod = common::Time(1.0/200);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ModelLocation::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ModelLocation::OnUpdate(const common::UpdateInfo &_info)
{
  if (this->world->GetSimTime() - this->lastUpdateTime < this->updatePeriod)
    return;

  this->lastUpdateTime = this->world->GetSimTime();

  if (this->posePublisher->HasConnections())
  {
    msgs::Pose msg;

    msg.set_name(this->model->GetName());

    // Dumb gazebo::math <-> ignition::math incompatibility
    auto gm_pose = this->model->GetWorldPose();
    ignition::math::Pose3d pose(gm_pose.pos.x,
                                gm_pose.pos.y,
                                gm_pose.pos.z,
                                gm_pose.rot.w,
                                gm_pose.rot.x,
                                gm_pose.rot.y,
                                gm_pose.rot.z);

    msgs::Set(msg.mutable_position(), pose.Pos());
    msgs::Set(msg.mutable_orientation(), pose.Rot());

    this->posePublisher->Publish(msg);
  }
}

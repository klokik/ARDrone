#ifndef _GAZEBO_MODEL_FORCE_TORQUE_HH_
#define _GAZEBO_MODEL_FORCE_TORQUE_HH_

#include <gazebo/msgs/vector3d.pb.h>
#include <vector>


namespace gazebo
{
  /// \brief A plugin to apply force and torque to the model
  class ModelForceTorque : public ModelPlugin
  {
    /// \brief Constructor
    public: ModelForceTorque();

    /// \brief Parse goals defined in the SDF
    /// \param[in] _sdf sdf pointer corresponding to goals element
    /// \return True if parsing was succesfull or not
    //private: bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    /// \brief Callback to run when recieve a push message.
    /// \param[in] _msg force vector3 message received.
    public: void OnForceMsg(ConstVector3dPtr &_msg);

    /// \brief Callback to run when recieve a push message.
    /// \param[in] _msg torque RPY message received.
    public: void OnTorqueMsg(ConstVector3dPtr &_msg);

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Pointer to the model that defines this plugin
    private: physics::ModelPtr model;

    /// \brief Pointer to the link of the model to be controlled
    private: physics::LinkPtr link;

    /// \brief Transport node used to communicate with the transport system
    private: transport::NodePtr node;

    /// \brief Subscriber to get force messages
    private: transport::SubscriberPtr forceSubscriber;

    /// \brief Subscriber to get torque messages
    private: transport::SubscriberPtr torqueSubscriber;

    /// \brief Last received force
    private: ignition::math::Vector3d lastForce;

    /// \brief Last received torque
    private: ignition::math::Vector3d lastTorque;

    /// \brief Pointer to the world
    protected: gazebo::physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
}
#endif

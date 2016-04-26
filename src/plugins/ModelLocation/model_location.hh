#ifndef _GAZEBO_MODEL_LOCATION_HH_
#define _GAZEBO_MODEL_LOCATION_HH_

#include <gazebo/msgs/pose.pb.h>
#include <vector>


namespace gazebo
{
  /// \brief A plugin to move a model to a specific pose and track
  /// it's position
  class ModelLocation : public ModelPlugin
  {
    /// \brief Constructor
    public: ModelLocation();

    /// \brief Parse goals defined in the SDF
    /// \param[in] _sdf sdf pointer corresponding to goals element
    /// \return True if parsing was succesfull or not
    //private: bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    /// \brief Callback to run when recieve a move message.
    /// \param[in] _msg pose message received to set model pose.
    public: void OnPoseMsg(ConstPosePtr &_msg);

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Pointer to the model that defines this plugin
    private: physics::ModelPtr model;

    /// \brief Transport node used to communicate with the transport system
    private: transport::NodePtr node;

    /// \brief Subscriber to get pose messages
    private: transport::SubscriberPtr poseSubscriber;

    /// \brief Publisher to post pose messages
    private: transport::PublisherPtr posePublisher;

    /// \brief Time of the last update
    private: common::Time lastUpdateTime;

    /// \brief Update period = 1s/(update rate)
    private: common::Time updatePeriod;

    /// \brief Pointer to the world
    protected: gazebo::physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
}
#endif

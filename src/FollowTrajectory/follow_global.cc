#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <cassert>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <mutex>
#include <queue>

#include "HelperMath.hh"


class UAVController
{
  protected: struct IMUFrame
  {
    gazebo::common::Time stamp;
    ignition::math::Quaterniond orientation; 
    ignition::math::Vector3d angular_vel;
    ignition::math::Vector3d linear_acc;
  };

  public: UAVController(std::string _model_name/*,
                        std::vector<std::string> _marker_name_list*/):
      model_name(_model_name)
  {
    this->z_pid.Init(
      200, 0.1, 10,  // P, I, D,
      2, 0,       // imax, imin,
      20, 0);     // cmd_max, cmd_min

    this->x_pid.Init(30, 0.1, 10, 0, 0, 10, -10);
    this->y_pid.Init(30, 0.1, 10, 0, 0, 10, -10);
  }

  public: auto getEstimatedPose()
  { return this->estimated_pose; }

  // [not implemented yet]
  public: void showDebugPose();

  public: void initTransport(gazebo::transport::NodePtr _tnode)
  {
    std::string prefix = "~/" + this->model_name;

    /*this->camera_sub = _tnode->Subscribe(prefix + "/camera/link/camera/image",
        &UAVController::cameraImageStampedMsg, this);*/
    this->pose_sub = _tnode->Subscribe(prefix + "/pose/info",
        &UAVController::uavPoseMsg, this);
    this->imu_sub = _tnode->Subscribe(prefix + "/quadrotor_link/imu_sensor/imu",
        &UAVController::uavIMUMsg, this);

    this->force_pub = _tnode->Advertise<gazebo::msgs::Vector3d>(prefix + "/force/add_force");
    // this->torque_pub = _tnode->Advertise<gazebo::msgs::Vector3d>(prefix + "/force/add_torque");
    this->debug_pose_pub = _tnode->Advertise<gazebo::msgs::Pose>(std::string("~/box/pose/modify"));
    this->pose_pub = _tnode->Advertise<gazebo::msgs::Pose>(prefix + "/pose/modify");
  }

  public: void setTargetPoint(ignition::math::Vector3d const &_targ)
  {
    this->target_position = _targ;
  }

  public: double distanceToTarget()
  {
    return (this->global_pose.Pos() - this->target_position).Length();
  }

  // TODO: Use pose_stamped
  protected: void uavPoseMsg(ConstPosePtr &_msg)
  {
    auto mpos = _msg->position();
    auto mrot = _msg->orientation();

    auto new_pose = ignition::math::Pose3d(
        ignition::math::Vector3d(mpos.x(), mpos.y(), mpos.z()),
        ignition::math::Quaterniond(mrot.w(), mrot.x(), mrot.y(), mrot.z()));

    if (_msg->name() == this->model_name)
      this->global_pose = new_pose;
    else {/*Ignore*/}
  }

  protected: void uavIMUMsg(ConstIMUPtr &_msg)
  {
    IMUFrame frame;

    frame.stamp = gazebo::common::Time(_msg->stamp().sec(), _msg->stamp().nsec());

    auto mquat = _msg->orientation();
    auto mavel = _msg->angular_velocity();
    auto mlacc = _msg->linear_acceleration();

    frame.orientation = ignition::math::Quaterniond(mquat.w(),
        mquat.x(), mquat.y(), mquat.z());

    frame.angular_vel = ignition::math::Vector3d(mavel.x(), mavel.y(), mavel.z());
    frame.linear_acc = ignition::math::Vector3d(mlacc.x(), mlacc.y(), mlacc.z());

    this->appendIMUFrame(frame);
  }

  public: void update(gazebo::common::Time const &_dt)
  {
    ignition::math::Vector3d force;
    // ignition::math::Vector3d torque;

    auto mdirection = this->global_pose.Pos() - this->target_position;
    auto distance = mdirection.Length();

    auto x_err = mdirection.X();
    auto y_err = mdirection.Y();
    auto z_err = mdirection.Z();

    auto z_cmd = z_pid.Update(z_err, _dt);
    auto x_cmd = x_pid.Update(x_err, _dt);
    auto y_cmd = y_pid.Update(y_err, _dt);

    force = ignition::math::Vector3d(x_cmd, y_cmd, z_cmd)*this->model_mass;

    auto dir_z = -mdirection.Normalize();
    auto dir_y = dir_z.Cross(this->global_pose.Rot().XAxis()).Normalize();
    auto dir_x = dir_y.Cross(dir_z).Normalize();

    ignition::math::Matrix3d directed_frame;
    directed_frame.Axes(dir_x, dir_y, dir_z);

    auto desired_orient = matrixToQuaternion(directed_frame);

    // reduce model tilt when it's closer to target
    auto iweight = std::exp(-distance/10);
    desired_orient = ignition::math::Quaterniond::Slerp(iweight,
        desired_orient, ignition::math::Quaterniond(0, 0, 0));

    this->applyForce(force);
    // this->applyTorque(torque);

    this->applyOrientation(desired_orient);
  }

  public: void applyForce(ignition::math::Vector3d const &_force)
  {
    gazebo::msgs::Vector3d msgv;

    msgv.set_x(_force.X());
    msgv.set_y(_force.Y());
    msgv.set_z(_force.Z());

    this->force_pub->Publish(msgv);
  }

  public: void applyOrientation(ignition::math::Quaterniond const &_q)
  {
    gazebo::msgs::Pose msgp;

    msgp.set_id(1); // keep position intact, update just orientation

    gazebo::msgs::Set(msgp.mutable_position(), ignition::math::Vector3d(0, 0, 0));
    gazebo::msgs::Set(msgp.mutable_orientation(), _q);

    this->pose_pub->Publish(msgp);
  }

  protected: void appendIMUFrame(IMUFrame const &_frame)
  {
    if (!this->imu_history.empty())
    {
      auto dt_v = _frame.stamp - this->imu_history.back().stamp;

      // THIS CODE IS PROBABLY WRONG
      this->estimated_speed += _frame.orientation*_frame.linear_acc*dt_v.Double();
      this->estimated_pose.Pos() += this->estimated_speed*dt_v.Double();
      // throw std::runtime_error("not tested code");
    }

    this->imu_history.push(_frame);

    while (this->imu_history.size() > this->imu_q_size)
      this->imu_history.pop();
  }

  protected: double model_mass = 1.316+0.1;
  protected: std::string model_name;

  protected: const int pose_q_size = 8;
  protected: const int imu_q_size = 100;
  protected: std::queue<std::pair<gazebo::common::Time, ignition::math::Pose3d>> pose_history;
  protected: std::queue<IMUFrame> imu_history;

  protected: ignition::math::Vector3d target_position;

  protected: ignition::math::Pose3d estimated_pose; // Cameras estimated pose + cumulative imu data
  protected: ignition::math::Pose3d global_pose;    // Beforehand known pose
  protected: ignition::math::Vector3d estimated_speed;

  protected: gazebo::common::PID x_pid;
  protected: gazebo::common::PID y_pid;
  protected: gazebo::common::PID z_pid;

  private: gazebo::transport::SubscriberPtr camera_sub;
  private: gazebo::transport::SubscriberPtr marker_sub;
  private: gazebo::transport::SubscriberPtr pose_sub;
  private: gazebo::transport::SubscriberPtr imu_sub;
  private: gazebo::transport::PublisherPtr force_pub;
  private: gazebo::transport::PublisherPtr torque_pub;
  private: gazebo::transport::PublisherPtr pose_pub;
  private: gazebo::transport::PublisherPtr debug_pose_pub;
};

struct Trajectory
{
  // length of the segment (in seconds, relative to the previous point) and position of the next point
  public: std::vector<std::pair<double, ignition::math::Vector3d>> waypoints;

  // current position on the path (in seconds)
  double position = 0;

  double length()
  {
    double len = 0;
    for (auto &item : waypoints)
      len += item.first;

    return len;
  }

  ignition::math::Vector3d getTrajectoryPoint(double const _t)
  {
    assert(!waypoints.empty());

    if (waypoints.size() == 1)
      return this->waypoints.begin()->second;

    double cum_length;
    for (int q = 1; q < waypoints.size(); ++q)
    {
      if (cum_length+waypoints[q].first < _t)
      {
        cum_length += waypoints[q].first;
        continue;
      }

      double iweight = (_t - cum_length)/waypoints[q].first;

      return (waypoints[q-1].second*(1-iweight) + waypoints[q].second*iweight);
    }

    return this->waypoints.rbegin()->second;
  }

  ignition::math::Vector3d getTrajectoryPoint()
  {
    return getTrajectoryPoint(this->position);
  }

  void addWaypoint(double const &_seg_dt, ignition::math::Vector3d const &_pos)
  {
    this->waypoints.push_back({_seg_dt, _pos});
  }
};

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ignition::math::Pose3d desired_pose(0, 2, 3, 0, 0, 0);

  UAVController uav_ctrl("quadrotor");

  uav_ctrl.initTransport(node);

  Trajectory traj;

  for (double phi = 0; phi < M_PI*2; phi += 0.1)
  {
    double R = 10;

    double r = std::cos(phi*3)*R;

    auto x = r * std::cos(phi);
    auto y = r * std::sin(phi);
    auto z = 2.0;

    traj.addWaypoint(1, ignition::math::Vector3d(x, y, z));
  }

  int32_t loop_time = 10;
  while (true)
  {
    gazebo::common::Time::MSleep(loop_time);
    auto dt = gazebo::common::Time(0, loop_time*1000000);

    auto target_point = desired_pose.Pos();

    // wait until quad is close to taget
    if (uav_ctrl.distanceToTarget() < 0.5)
      traj.position += 1;

    uav_ctrl.setTargetPoint(traj.getTrajectoryPoint());
    uav_ctrl.update(dt);
  }

  std::cout << "Shutting down" << std::endl;
  gazebo::client::shutdown();
}

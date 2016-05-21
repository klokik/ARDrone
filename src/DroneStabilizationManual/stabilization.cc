#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <cassert>
#include <cstring>
#include <iostream>
#include <mutex>


gazebo::common::Time last_pos_time;
ignition::math::Pose3d quad_pose;

bool has_new_position = false;


void poseMsg(ConstPosePtr &_msg)
{
  last_pos_time = gazebo::common::Time::GetWallTime();

  auto mpos = _msg->position();
  auto mrot = _msg->orientation();

  auto new_pose = ignition::math::Pose3d(
      ignition::math::Vector3d(mpos.x(), mpos.y(), mpos.z()),
      ignition::math::Quaterniond(mrot.w(), mrot.x(), mrot.y(), mrot.z()));

  if (_msg->name() == "quadrotor")
  {
    quad_pose = new_pose;
    has_new_position = true;
  }
  else {/*Ignore*/}
}

ignition::math::Quaterniond matrixToQuaternion(ignition::math::Matrix3d const &_m)
{
  double tr = _m(0,0) + _m(1,1) + _m(2,2);

  double qw = 0;
  double qx = 0;
  double qy = 0;
  double qz = 0;

  if (tr > 0) {
    double S = std::sqrt(tr+1.0) * 2; // S=4*qw 
    qw = 0.25 * S;
    qx = (_m(2,1) - _m(1,2)) / S;
    qy = (_m(0,2) - _m(2,0)) / S; 
    qz = (_m(1,0) - _m(0,1)) / S; 
  } else if ((_m(0,0) > _m(1,1))&(_m(0,0) > _m(2,2))) {
    double S = std::sqrt(1.0 + _m(0,0) - _m(1,1) - _m(2,2)) * 2; // S=4*qx 
    qw = (_m(2,1) - _m(1,2)) / S;
    qx = 0.25 * S;
    qy = (_m(0,1) + _m(1,0)) / S; 
    qz = (_m(0,2) + _m(2,0)) / S; 
  } else if (_m(1,1) > _m(2,2)) {
    double S = std::sqrt(1.0 + _m(1,1) - _m(0,0) - _m(2,2)) * 2; // S=4*qy
    qw = (_m(0,2) - _m(2,0)) / S;
    qx = (_m(0,1) + _m(1,0)) / S; 
    qy = 0.25 * S;
    qz = (_m(1,2) + _m(2,1)) / S; 
  } else {
    double S = std::sqrt(1.0 + _m(2,2) - _m(0,0) - _m(1,1)) * 2; // S=4*qz
    qw = (_m(1,0) - _m(0,1)) / S;
    qx = (_m(0,2) + _m(2,0)) / S;
    qy = (_m(1,2) + _m(2,1)) / S;
    qz = 0.25 * S;
  }

  return {qw, qx, qy, qz};
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  auto quad_pose_sub = node->Subscribe("~/quadrotor/pose/info", poseMsg);
  auto quad_pose_pub = node->Advertise<gazebo::msgs::Pose>("~/quadrotor/pose/modify");

  std::string quad_force_address = "~/quadrotor/force/add_force";
  std::string quad_torque_address = "~/quadrotor/force/add_torque";
  auto quad_force_pub = node->Advertise<gazebo::msgs::Vector3d>(quad_force_address);
  auto quad_torque_pub = node->Advertise<gazebo::msgs::Vector3d>(quad_torque_address);
  std::cout << "Altering `quadrotor` model location on " << quad_force_address << std::endl;


  gazebo::common::PID altitude_pid(
      200, 0.1, 10,  // P, I, D,
      2, 0,       // imax, imin,
      20, 0);     // cmd_max, cmd_min

  gazebo::common::PID x_pid(30, 0.1, 10, 0, 0, 10, -10);
  gazebo::common::PID y_pid(30, 0.1, 10, 0, 0, 10, -10);

  double rp_p = 1, rp_i = 0.1, rp_d = 1;
  double yaw_p = 1, yaw_i = 0.1, yaw_d = 1;

  gazebo::common::PID yaw_pid(yaw_p, yaw_i, yaw_d, 0, 0, 1, -1);
  gazebo::common::PID pitch_pid(rp_p, rp_i, rp_d, 0, 0, 1, -1);
  gazebo::common::PID roll_pid (rp_p, rp_i, rp_d, 0, 0, 1, -1);

  double quad_mass = 1.316 + 0.1;

  ignition::math::Pose3d desired_pose(0, 2, 3, 0, 0, 0);

  int32_t ms_loop = 10;
  float dt_sum = 0;
  while (true)
  {
    gazebo::common::Time::MSleep(ms_loop);
    dt_sum += ms_loop;

    if (!has_new_position)
      continue;
    else
      has_new_position = false;

    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;

    double sign = 1;

    auto mdirection = quad_pose.Pos() - desired_pose.Pos();
    auto distance = mdirection.Length();

    auto dt = gazebo::common::Time(0, dt_sum*1000000);
    auto x_err = mdirection.X();
    auto y_err = mdirection.Y();
    auto z_err = mdirection.Z();

    double z_cmd = altitude_pid.Update(z_err, dt);
    double x_cmd = x_pid.Update(x_err, dt);
    double y_cmd = y_pid.Update(y_err, dt);

    force = ignition::math::Vector3d(x_cmd, y_cmd, z_cmd)*quad_mass;

    std::cout << "Pid: (" << z_cmd << "," << x_cmd+y_cmd << ")\tErr: " << z_err+x_err+y_err << std::endl;

    auto dir_z = -mdirection.Normalize();
    auto dir_y = dir_z.Cross(quad_pose.Rot().XAxis()).Normalize();
    auto dir_x = dir_y.Cross(dir_z).Normalize();

    ignition::math::Matrix3d directed_frame;
    directed_frame.Axes(dir_x, dir_y, dir_z);

    auto desired_orient = matrixToQuaternion(directed_frame);

    // reduce model tilt when it's closer to target
    auto iweight = std::exp(-distance/10);
    desired_orient = ignition::math::Quaterniond::Slerp(iweight,
        desired_orient, ignition::math::Quaterniond(0, 0, 0));

/*    auto orientation_difference = quad_pose.Rot().Inverse()*desired_orient;  //q1^-1 * q2
    // auto dyaw    = desired_orient.Yaw();
    // auto dpitch  = desired_orient.Pitch();
    // auto droll   = desired_orient.Roll();

    // auto yaw    = quad_pose.Rot().Yaw();
    // auto pitch  = quad_pose.Rot().Pitch();
    // auto roll   = quad_pose.Rot().Roll();

    auto yaw    = orientation_difference.Yaw();
    auto pitch  = orientation_difference.Pitch();
    auto roll   = orientation_difference.Roll();

    // if (dyaw   > M_PI)    dyaw = (dyaw   - 2*M_PI);
    // if (dpitch > M_PI)  dpitch = (dpitch - 2*M_PI);
    // if (droll  > M_PI)   droll = (droll  - 2*M_PI);

    if (yaw   > M_PI)    yaw = (yaw   - 2*M_PI);
    if (pitch > M_PI)  pitch = (pitch - 2*M_PI);
    if (roll  > M_PI)   roll = (roll  - 2*M_PI);

    // std::cout << "RPY: (" << droll << "\t" << dpitch << "\t" << dyaw << ")" << std::endl;

    // double   yaw_cmd =   yaw_pid.Update(yaw - dyaw, dt);
    // double pitch_cmd = pitch_pid.Update(pitch - dpitch, dt);
    // double  roll_cmd =  roll_pid.Update(roll - droll, dt);*/

    auto tx = quad_pose.Rot().XAxis().Cross(desired_orient.XAxis());
    auto ty = quad_pose.Rot().YAxis().Cross(desired_orient.YAxis());

    // double   yaw_cmd =   yaw_pid.Update(yaw, dt);
    double pitch_cmd = pitch_pid.Update(ty.Length(), dt);
    double  roll_cmd =  roll_pid.Update(tx.Length(), dt);

    torque = (tx*roll_cmd + ty*pitch_cmd);

    gazebo::msgs::Vector3d msgv;

    msgv.set_x(force.X());
    msgv.set_y(force.Y());
    msgv.set_z(force.Z());

    quad_force_pub->Publish(msgv);

    msgv.set_x(torque.X());
    msgv.set_y(torque.Y());
    msgv.set_z(torque.Z());

    // quad_torque_pub->Publish(msgv);

    gazebo::msgs::Pose msgp;
    msgp.set_id(1); // keep position intact, update just orientation
    gazebo::msgs::Set(msgp.mutable_position(), quad_pose.Pos());
    gazebo::msgs::Set(msgp.mutable_orientation(), desired_orient);
    quad_pose_pub->Publish(msgp);
  }

  std::cout << "Shutting down" << std::endl;
  gazebo::client::shutdown();
}

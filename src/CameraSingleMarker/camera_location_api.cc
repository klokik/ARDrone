#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <cassert>
#include <cstring>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

// #include "CameraLocator.hh"
#include "Locator.cc"


std::mutex image_mutex;
cv::Mat cam_mat;

ignition::math::Pose3d camera_pose_global;
ignition::math::Pose3d marker_pose_global;

auto markers = std::make_shared<ArFeatures>();

void cb(ConstImageStampedPtr &_msg)
{
  auto step = _msg->image().step();
  auto width = _msg->image().width();
  auto height = _msg->image().height();

  // std::cout << "Data size: " << _msg->image().data().size() << "\n size: "
  //           << width << "x" << height
  //           << "\n pixel_format: " << gazebo::common::PixelFormatNames[_msg->image().pixel_format()]
  //           << "\n step: " << _msg->image().step() << std::endl;

  image_mutex.lock();

  assert(_msg->image().pixel_format() == 3); // Invalid pixel format
  if (cam_mat.rows != height || cam_mat.cols != width)
    cam_mat = cv::Mat(height, width, CV_8UC3);

  for (size_t irow = 0; irow < height; ++irow)
  {
    auto row_ptr = cam_mat.ptr(irow);
    std::memcpy(row_ptr, &_msg->image().data()[irow*step], step);
  }
  cv::cvtColor(cam_mat, cam_mat, CV_BGR2RGB);

  image_mutex.unlock();
}

void poseMsg(ConstPosePtr &_msg)
{
  auto mpos = _msg->position();
  auto mrot = _msg->orientation();

  auto new_pose = ignition::math::Pose3d(
      ignition::math::Vector3d(mpos.x(), mpos.y(), mpos.z()),
      ignition::math::Quaterniond(mrot.w(), mrot.x(), mrot.y(), mrot.z()));

  if (_msg->name() == "camera_0")
    camera_pose_global = new_pose;
  else if(_msg->name() == "ar_diamond_0")
  {
    marker_pose_global = new_pose;
    markers->operator[](cv::Vec4i(1, 2, 3, 4)) = marker_pose_global;
  }
  else {/*Ignore*/}
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to camera image topic
  gazebo::transport::SubscriberPtr sub =
      node->Subscribe("~/camera_0/camera_0/link/camera/image", cb);

  gazebo::transport::SubscriberPtr pose_sub_cam =
      node->Subscribe("~/camera_0/pose/info", poseMsg);

  gazebo::transport::SubscriberPtr pose_sub_diam =
      node->Subscribe("~/ar_diamond_0/pose/info", poseMsg);

  std::string box_mod_address = "~/box/pose/modify";
  gazebo::transport::PublisherPtr box_pub =
      node->Advertise<gazebo::msgs::Pose>(box_mod_address);
  gzmsg << "Altering `box` model location on " << box_mod_address << std::endl;

  std::string camera_mod_address = "~/camera_0/pose/modify";
  gazebo::transport::PublisherPtr camera_0_pub =
      node->Advertise<gazebo::msgs::Pose>(camera_mod_address);
  gzmsg << "Altering `camera_0` model location on " << camera_mod_address << std::endl;

  std::string marker_mod_address = "~/ar_diamond_0/pose/modify";
  gazebo::transport::PublisherPtr marker_pub =
      node->Advertise<gazebo::msgs::Pose>(marker_mod_address);
  gzmsg << "Altering `ar_diamond_0` model location on " << marker_mod_address << std::endl;


  ArLocator locator(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));
  float distortion[] = {0, 0, 0, 0, 0};
  locator.addCamera(0, CameraProperties(1.047, 640, 640, distortion));

  locator.setWorldFeatures(markers);

  float t = 0;
  int frames = 10;
  while (true)
  {
    gazebo::common::Time::MSleep(10);

    t += 0.01;
    ignition::math::Pose3d box_pose;

    image_mutex.lock();
    if (cam_mat.cols > 0 && cam_mat.rows > 0)
    {
      // box_pose = locateCamera(cam_mat, marker_pose_global);
      locator.feedDataFrame(cam_mat, gazebo::common::Time::GetWallTime());

      box_pose = locator.getEstimatedPosition(gazebo::common::Time::GetWallTime());
      frames--;

      std::cout << "Err: " << (box_pose.Pos() - camera_pose_global.Pos()).Length()
                << std::endl;

      gazebo::msgs::Pose msgp;

      msgp.set_name("box");
      gazebo::msgs::Set(msgp.mutable_position(), box_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), box_pose.Rot());

      box_pub->Publish(msgp);

      ignition::math::Pose3d cam_pose =
          ignition::math::Pose3d(-2+std::sin(t), 0, 0, 0, 0, 0)+
          ignition::math::Pose3d(0, 0, 0, 0, M_PI/3, t);

      msgp.set_name("camera_0");
      gazebo::msgs::Set(msgp.mutable_position(), cam_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), cam_pose.Rot());

      camera_0_pub->Publish(msgp);

      ignition::math::Pose3d mark_pose =
          ignition::math::Pose3d(0, 0, 0, 0, std::sin(t/2), 0);

      msgp.set_name("ar_diamond_0");
      gazebo::msgs::Set(msgp.mutable_position(), mark_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), mark_pose.Rot());

      marker_pub->Publish(msgp);

      cv::imshow("cam0", cam_mat);
    }
    image_mutex.unlock();

    if (cv::waitKey(16) == 27)
      break;
  }

  gazebo::client::shutdown();
}

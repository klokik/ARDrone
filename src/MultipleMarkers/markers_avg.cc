#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <fstream>
#include <cassert>
#include <cstring>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "Locator.cc"


std::mutex image_mutex;
cv::Mat cam_mat;
gazebo::common::Time last_frame_time;

ignition::math::Pose3d camera_pose_global;

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

  last_frame_time = gazebo::common::Time(_msg->time().sec(), _msg->time().nsec());

  image_mutex.unlock();
}

void poseMsg(ConstPoseStampedPtr &_msg)
{
  auto new_pose = gazebo::msgs::ConvertIgn(_msg->pose());

  std::string p_name = _msg->pose().name();

  if (p_name == "camera_0")
    camera_pose_global = new_pose;
  else if(p_name == "ar_diamond_0")
    markers->operator[](cv::Vec4i(0, 1, 2, 3)) = new_pose;
  else if(p_name == "ar_diamond_4")
    markers->operator[](cv::Vec4i(4, 5, 6, 7)) = new_pose;
  else if(p_name == "ar_diamond_8")
    markers->operator[](cv::Vec4i(8, 9, 10, 11)) = new_pose;
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

  auto pose_sub_diam1 = node->Subscribe("~/ar_diamond_0/pose/info", poseMsg);
  // auto pose_sub_diam5 = node->Subscribe("~/ar_diamond_4/pose/info", poseMsg);
  // auto pose_sub_diam9 = node->Subscribe("~/ar_diamond_8/pose/info", poseMsg);

  std::string box_mod_address = "~/box/pose/modify";
  gazebo::transport::PublisherPtr box_pub =
      node->Advertise<gazebo::msgs::Pose>(box_mod_address);
  gzmsg << "Altering `box` model location on " << box_mod_address << std::endl;

  std::string camera_mod_address = "~/camera_0/pose/modify";
  gazebo::transport::PublisherPtr camera_0_pub =
      node->Advertise<gazebo::msgs::Pose>(camera_mod_address);
  gzmsg << "Altering `camera_0` model location on " << camera_mod_address << std::endl;


  ArLocator locator(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));
  float distortion[] = {0, 0, 0, 0, 0};
  locator.addCamera(0, CameraProperties(1.047, 640, 640, distortion));

  locator.setWorldFeatures(markers);

  // std::ofstream ofs("./logs/length-err.dat");
  std::ofstream ofs("./logs/length-err-single.dat");

  std::vector<double> errors;

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
      locator.feedDataFrame(cam_mat, last_frame_time);

      box_pose = locator.getEstimatedPosition(gazebo::common::Time::GetWallTime());
      frames--;

      double err = (box_pose.Pos() - camera_pose_global.Pos()).Length();

      if (err < 0.3)
        errors.push_back(err);

      ofs << camera_pose_global.Pos().Length() << " "
          << err << std::endl;
      std::cout << "Err: " << err << std::endl;

      gazebo::msgs::Pose msgp;

      msgp.set_name("box");
      gazebo::msgs::Set(msgp.mutable_position(), box_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), box_pose.Rot());

      box_pub->Publish(msgp);

      auto cam_pose =
          ignition::math::Pose3d(-2+std::sin(t), 0, 0, 0, 0, 0) +
          ignition::math::Pose3d(0, 0, 0, 0, M_PI/3, t);

      msgp.set_name("camera_0");
      gazebo::msgs::Set(msgp.mutable_position(), cam_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), cam_pose.Rot());

      camera_0_pub->Publish(msgp);

      cv::resize(cam_mat, cam_mat, cv::Size(240, 240));
      cv::imshow("cam0", cam_mat);
    }
    image_mutex.unlock();

    if (cv::waitKey(16) == 27)
      break;
  }
  ofs.close();

  double sum = 0;
  for (auto item : errors)
    sum += item*item;

  std::cout << "Msq: " << std::sqrt(sum/errors.size()) << std::endl;

  gazebo::client::shutdown();
}

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


std::mutex image_mutex;
cv::Mat cam_mat;

ignition::math::Pose3<double> camera_pose_global;
ignition::math::Pose3<double> marker_pose_global;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
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
    marker_pose_global = new_pose;
  else {/*Ignore*/}
}

ignition::math::Pose3<double> convertFrame(cv::Vec3d _pos, cv::Vec3d _rot)
{
  using namespace ignition::math;

  // Matrix3<double> perm( 0,-1, 0,
  //                      -1, 0, 0,
  //                       0, 0,-1);

  auto S = Matrix3<double>::Identity; //perm;

  Vector3<double> position(_pos[0], _pos[1], _pos[2]);
  Vector3<double> rodrigues(_rot[0], _rot[1], _rot[2]);
  float angle = rodrigues.Length();
  auto axis = rodrigues.Normalize();

  axis = S * axis;
  position = S * position;

  return Pose3<double>(-position, Quaterniond(axis, angle*0))+
         Pose3<double>(Vector3<double>(0, 0, 0), Quaterniond(axis, -angle));
}

ignition::math::Pose3<double> locateCamera(cv::Mat &mat)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, diamond_corners;
  std::vector<cv::Vec4i> diamond_ids; 
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  cv::aruco::detectMarkers(mat, dictionary, marker_corners, ids);
  if (ids.size() > 0)
    cv::aruco::detectCharucoDiamond(mat, marker_corners, ids,
                                    128/96.f, diamond_corners, diamond_ids);

  cv::Mat mat_copy;
  mat.copyTo(mat_copy);

  if (ids.size() > 0)
    cv::aruco::drawDetectedMarkers(mat_copy, marker_corners, ids);

  ignition::math::Pose3<double> camera_pose(0, 0, 1.5, 0, 1.5708, 0); // FALLBACK

  std::vector<cv::Vec3d> rvecs, tvecs;
  if (diamond_ids.size() > 0)
  {
    float square_size = 0.1;

    double hfov = 1.047;  // 60 degrees
    double fx = mat.cols/std::tan(hfov/2)/2;
    double fy = fx*mat.rows/mat.cols;


    cv::Mat prj_matrix = (cv::Mat_<double>(3, 3) << fx, 0,  mat.cols/2.,
                                                    0,  fy, mat.rows/2.,
                                                    0,  0,  1);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_32F);

    cv::aruco::estimatePoseSingleMarkers(diamond_corners, square_size,
        prj_matrix, dist_coeffs, rvecs, tvecs);

    for (size_t i = 0; i< diamond_ids.size(); ++i)
      cv::aruco::drawAxis(mat_copy, prj_matrix, dist_coeffs,
          rvecs[i], tvecs[i], square_size*2);
    cv::aruco::drawDetectedDiamonds(mat_copy, diamond_corners, diamond_ids);

    assert(diamond_ids.size() == 1);

    auto camera_pose_marker_frame = convertFrame(tvecs[0], rvecs[0]);

    camera_pose = camera_pose_marker_frame + marker_pose_global;
    std::cout << camera_pose << std::endl;
  }

  cv::imshow("camera_0", mat_copy);

  return camera_pose;
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

/*  auto *database = gazebo::common::ModelDatabase::Instance();
  auto models = database->GetModels();
  for (auto model : models)
    std::cout << model.first << " : " << model.second << std::endl;*/

  std::string box_mod_address = "~/box/pose/modify";
  gazebo::transport::PublisherPtr pub =
      node->Advertise<gazebo::msgs::Pose>(box_mod_address);
  gzmsg << "Altering `box` model location on " << box_mod_address << std::endl;

  float t = 0;
  int frames = 10;
  while (true)//frames > 0)
  {
    gazebo::common::Time::MSleep(10);

    t += 0.01;
    ignition::math::Pose3<double> box_pose(
        ignition::math::Vector3d(0, 0, 3-std::sin(t)),
        ignition::math::Quaterniond(t, 0, 0));

    image_mutex.lock();
    if (cam_mat.cols > 0 && cam_mat.rows > 0)
    {
      box_pose = locateCamera(cam_mat);
      frames--;

      gazebo::msgs::Pose msgp;

      msgp.set_name("box");
      gazebo::msgs::Set(msgp.mutable_position(), box_pose.Pos());
      gazebo::msgs::Set(msgp.mutable_orientation(), box_pose.Rot());

      pub->Publish(msgp);
    }
    image_mutex.unlock();

    if (cv::waitKey(16) == 27)
      break;
  }

  gazebo::client::shutdown();
}

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


/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstImageStampedPtr &_msg)
{
  auto step = _msg->image().step();
  auto width = _msg->image().width();
  auto height = _msg->image().height();

  std::cout << "Data size: " << _msg->image().data().size() << "\n size: "
            << width << "x" << height
            << "\n pixel_format: " << gazebo::common::PixelFormatNames[_msg->image().pixel_format()]
            << "\n step: " << _msg->image().step() << std::endl;

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

void locateCamera(cv::Mat &mat)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, diamond_corners;
  std::vector<cv::Vec4i> diamond_ids; 
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  cv::aruco::detectMarkers(mat, dictionary, marker_corners, ids);
  if (ids.size() > 0)
    cv::aruco::detectCharucoDiamond(mat, marker_corners, ids,
                                    128/96.f, diamond_corners, diamond_ids);

  cv::Mat mat_copy;
  mat.copyTo(mat_copy);

  // if (ids.size() > 0)
    // cv::aruco::drawDetectedMarkers(mat_copy, marker_corners, ids);

  std::vector<cv::Vec3d> rvecs, tvecs;
  if (diamond_ids.size() > 0)
  {
    float square_size = 0.1;

    double hfov = 1.047;  // 60 degrees
    double fx = mat.cols*std::tan(hfov/2)/2;
    double fy = fx*mat.rows/mat.cols;


    cv::Mat prj_matrix = (cv::Mat_<double>(3, 3) << fx, 0,  mat.cols/2.,
                                                    0,  fy, mat.rows/2.,
                                                    0,  0,  1);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_32F);

    cv::aruco::estimatePoseSingleMarkers(diamond_corners, square_size,
        prj_matrix, dist_coeffs, rvecs, tvecs);

    for (size_t i = 0; i< diamond_ids.size(); ++i)
      cv::aruco::drawAxis(mat_copy, prj_matrix, dist_coeffs, rvecs[i], tvecs[i], square_size/2);
    cv::aruco::drawDetectedDiamonds(mat_copy, diamond_corners, diamond_ids);

    assert(diamond_ids.size() == 1);
    float x = tvecs[0][2]*3;
    float y = -tvecs[0][0];
    float z = -tvecs[0][1];
    tvecs[0][0] = x; tvecs[0][1] = y; tvecs[0][2] = z;
    std::cout << "pos " << tvecs[0] << std::endl;

    ignition::math::Pose3<double> mark_pose_local(tvecs[0][0], tvecs[0][1], tvecs[0][2],
                                                  rvecs[0][0], rvecs[0][1], rvecs[0][2]);
    ignition::math::Pose3<double> mark_pose_global(0.619463, 0.072396, 0.01, 0, 0, 0);
    ignition::math::Pose3<double> camera_pose(0.229545, -1.18587, 1.17264,
                                              1.2e-5, 0.721964, 1.38702);

    // float roll = 1.2e-5;
    // float pitch = 0.721964;
    // float yaw = 1.38702;
    // float c1 = std::cos(pitch), s1 = std::sin(pitch);
    // cv::Mat pitchm = (cv::Mat_<double>(3, 3) << c1, 0, -s1, 0, 1, 0, s1, 0, c1);
    // float c2 = std::cos(yaw), s2 = std::sin(yaw);
    // cv::Mat yawm = (cv::Mat_<double>(3, 3) << c2, s2, 0, -s2, c2, 0, 0, 0, 1);
    // float c3 = std::cos(roll), s3 = std::sin(roll);
    // cv::Mat rollm = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, c3, s3, 0, -s3, c3);

    // cv::Mat RPY_mat = rollm * pitchm * yawm;
    // cv::Mat RPYinv = RPY_mat.inv();

    // cv::Mat mark_loc_vec = (cv::Mat_<double>(3, 1) << tvecs[0][0], tvecs[0][1], tvecs[0][2]);
    // cv::Mat mark_loc = RPYinv*mark_loc_vec;

    // cv::Mat cam_glob_vec = (cv::Mat_<double>(3, 1) << 0.229545, -1.18587, 1.17264);
    // std::cout << cam_glob_vec+mark_loc << std::endl;

    std::cout << (mark_pose_local + camera_pose) << std::endl;
  }

  cv::imshow("camera_0", mat_copy);
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
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/camera_0/camera_0/link/camera/image", cb);

/*  auto *database = gazebo::common::ModelDatabase::Instance();
  auto models = database->GetModels();
  for (auto model : models)
    std::cout << model.first << " : " << model.second << std::endl;*/

  int frames = 10;
  while (frames > 0)
  {
    gazebo::common::Time::MSleep(10);

    // do some stuff
    image_mutex.lock();
    if (cam_mat.cols > 0 && cam_mat.rows > 0)
    {
      locateCamera(cam_mat);
      frames--;
      // cv::imshow("camera_0", cam_mat);
    }
    image_mutex.unlock();
    if (cv::waitKey(16) == 27)
      break;
  }

  gazebo::client::shutdown();
}

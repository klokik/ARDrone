#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>


ignition::math::Pose3d convertFrame(cv::Vec3d _pos, cv::Vec3d _rot)
{
  using namespace ignition::math;

  auto S = Matrix3d::Identity;

  Vector3d position(_pos[0], _pos[1], _pos[2]);
  Vector3d rodrigues(_rot[0], _rot[1], _rot[2]);
  float angle = rodrigues.Length();
  auto axis = rodrigues.Normalize();

  axis = S * axis;
  position = S * position;

  return Pose3d(-position, Quaterniond(axis, angle*0))+
         Pose3d(Vector3d(0, 0, 0), Quaterniond(axis, -angle));
}

ignition::math::Pose3d locateCamera(cv::Mat &mat,
	ignition::math::Pose3d &marker_pose_global)
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

  ignition::math::Pose3d camera_pose(0, 0, 1.5, 0, 1.5708, 0); // FALLBACK

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

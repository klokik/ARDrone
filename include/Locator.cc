#include <map>
#include <unordered_map>
#include <vector>
#include <memory>

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo/common/common.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

using Time_t = gazebo::common::Time;

template <class WorldFeatures, class InputData>
class Locator
{
  public: void setWorldFeatures(std::shared_ptr<WorldFeatures> _features)
  {
    this->world_features = _features;
  }

  public: virtual void feedDataFrame(InputData const &_data, Time_t const _time) = 0;

  public: virtual ignition::math::Pose3d getEstimatedPosition(Time_t _time)
  {
    std::vector<ignition::math::Vector3d> positions;
    std::vector<ignition::math::Quaterniond> orientations;
    std::vector<double> weights;
    double weight_sum = 0;

    for (auto &item : *this->world_features)
    {
      ignition::math::Pose3d single_pose;
      double weight;

      std::tie<ignition::math::Pose3d, double>(single_pose, weight) =
          this->estimatePositionSingleFeature(item);

      positions.push_back(single_pose.Pos());
      orientations.push_back(single_pose.Rot());
      weights.push_back(weight);

      weight_sum += weight;
    }

    ignition::math::Pose3d pose_avg;

    double weight_sum_partial = 0;

    for (size_t i = 0; i < weights.size(); ++i)
    {
      if (weights[i] < 1e-6)
        continue;

      double iweight = weight_sum_partial/(weight_sum_partial+weights[i]);
      weight_sum_partial += weights[i];

      pose_avg.Pos() = pose_avg.Pos()*iweight + positions[i]*(1-iweight);
      pose_avg.Rot() = ignition::math::Quaterniond::Slerp(iweight, pose_avg.Rot(), orientations[i]);
    }

    pose_avg.Rot().Normalize();

    return pose_avg;
  }

  // Returns estimated position and confidence coefficient
  protected: virtual std::pair<ignition::math::Pose3d, double>
      estimatePositionSingleFeature(
        typename WorldFeatures::value_type _feature) = 0;

  protected: std::shared_ptr<WorldFeatures> world_features;
};


class CameraProperties
{
  public: CameraProperties() = default;

  public: CameraProperties(float const _hfov, float const _sensor_width,
                           float const _sensor_height,
                           float _distortion[5]):
      hfov(_hfov),
      sensor_width(_sensor_width),
      sensor_height(_sensor_height)
  {
    distortion = cv::Mat(1, 5, CV_32F, static_cast<void*>(&_distortion[0]));
  }

  public: cv::Mat getCameraMatrix()
  {
    auto w = this->sensor_width;
    auto h = this->sensor_height;

    double fx = w/std::tan(this->hfov/2)/2;
    double fy = fx*h/w;

    cv::Mat prj_matrix = (cv::Mat_<double>(3, 3) << fx, 0,  w/2.,
                                                    0,  fy, h/2.,
                                                    0,  0,  1);

    return prj_matrix;
  }

  public: cv::Mat &getDistortion()
  {
    return this->distortion;
  }

  protected: float hfov = 1.047;
  protected: float sensor_width = 320;
  protected: float sensor_height = 320;
  protected: cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32F);
};

// Smallest marker id is used as a diamond hash value,
// assuming that each marker is used just on a single diamond
namespace std
{
  template <>
  struct hash<cv::Vec4i>
  {
    size_t operator()(cv::Vec4i const &_key) const noexcept
    {
      return std::min(std::min(_key[0], _key[1]), std::min(_key[2], _key[3]));
    }
  };

  template <>
  struct equal_to<cv::Vec4i>
  {
    bool operator()(cv::Vec4i const &_a, cv::Vec4i const &_b) const noexcept
    {
      // FIXME: sort and compare
      std::hash<cv::Vec4i> hash;
      return hash(_a) == hash(_b);
    }
  };
}

using ArFeatures = std::unordered_map<cv::Vec4i, ignition::math::Pose3d>;

class ArLocator: public Locator<ArFeatures, cv::Mat>
{
  using CameraId = int;

  struct LastFrameData
  {
    cv::Mat image;
    Time_t time;
    CameraId cam_id;

    std::unordered_map<cv::Vec4i, std::vector<cv::Point2f>> diamonds;

    // TODO: provide exhaustive marker information for the frame
    const float marker_width = 96;
    const float square_width = 128;
    const float diamond_rate = square_width/marker_width;

    const float square_length = 0.1f;
  };

  public: ArLocator(cv::aruco::Dictionary const &_dictionary):
      dictionary(_dictionary)
  {
  }

  public: virtual void feedDataFrame(cv::Mat const &_data, Time_t const _time)
  {
    last_input_frame.image = _data;
    last_input_frame.time = _time;

    last_input_frame.diamonds.clear();

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(_data, this->dictionary, marker_corners, marker_ids);


    std::vector<cv::Vec4i> diamond_ids;
    std::vector<std::vector<cv::Point2f>> diamond_corners;


    if (marker_ids.size() > 0)
    {
      // TODO: feed reprojection matrix
      cv::aruco::detectCharucoDiamond(_data, marker_corners, marker_ids,
          this->last_input_frame.diamond_rate,
          diamond_corners, diamond_ids);

      for (size_t i = 0; i < diamond_ids.size(); ++i)
        this->last_input_frame.diamonds[diamond_ids[i]] = diamond_corners[i];
    }
  }

  public: void addCamera(CameraId const _id, CameraProperties const &_props)
  {
    this->cameras[_id] = _props;
  }

  protected: std::pair<ignition::math::Pose3d, double>
      estimatePositionSingleFeature(
      std::pair<const cv::Vec4i, ignition::math::Pose3d> _feature) override
  {
    std::vector<std::vector<cv::Point2f>> diamond_corners;
    try
    {
      diamond_corners.push_back(
        this->last_input_frame.diamonds.at(_feature.first));
    }
    catch(std::exception const &ex)
    {
      ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);
      double confidence = 0;
      return {pose, confidence};
    }
    auto camera_props = this->cameras.at(0);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(diamond_corners,
        this->last_input_frame.square_length,
        camera_props.getCameraMatrix(), camera_props.getDistortion(),
        rvecs, tvecs);

    auto camera_pose_marker_frame = this->convertFrame(tvecs[0], rvecs[0]);

    auto est_pose = (camera_pose_marker_frame + _feature.second);
    double confidence = 1;
    return {est_pose, confidence};
  }

  private: ignition::math::Pose3d convertFrame(cv::Vec3d _pos, cv::Vec3d _rot)
  {
    using namespace ignition::math;

    Vector3d position(_pos[0], _pos[1], _pos[2]);
    Vector3d rodrigues(_rot[0], _rot[1], _rot[2]);
    float angle = rodrigues.Length();
    auto axis = rodrigues.Normalize();

    return Pose3d(-position, Quaterniond(axis, 0)) +
           Pose3d(Vector3d(0, 0, 0), Quaterniond(axis, -angle));
  }

  public: std::map<CameraId, CameraProperties> cameras;

  protected: cv::aruco::Dictionary dictionary;

  private: LastFrameData last_input_frame;
};
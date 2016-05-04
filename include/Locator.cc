
#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>


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

      pose_avg.Pos() = pose_avg.Pos()*iweight + inpositions[i]*(1-iweight);
      pose_avg.Rot() = ignition::math::Quaterniond::Slerp(iweight, pose_avg.Rot(), orientations[i]);
    }

    pose_avg.Rot() = pose_avg.Rot().Normalise();

    return pose_avg;
  }

  protected: virtual ignition::math::Pose3d estimatePositionSingle(WorldFeatures::value_type _feature) = 0;

  protected: std::shared_ptr<WorldFeatures> world_features;
};


using ArFeatures = std::unordered_map<cv::Vec4i, ignition::math::Pose3d>;

class ArLocator: public Locator<ArFeatures, cv::Mat>
{
  struct LastFrameData
  {
    cv::Mat image;
    Time_t time;

    std::unordered_map<cv::Vec4i, std::vector<cv::Point2f>> diamonds;

    const float marker_width = 96;
    const float square_width = 128;
    const float diamond_rate = marker_width/square_width;

    const float square_length = 0.1f;
  };

  public: ArLocator(cv::aruco::Dictionary &_dictionary):
      dictionary(_dictionary)
  {
  }

  public: virtual void feedDataFrame(cv::Mat const &_data, Time_t const _time)
  {
    last_input_frame.image = _data;
    last_input_frame.time = _time;

    last_input_frame.diamond_ids.clear();
    last_input_frame.diamond_corners.clear();

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(_data, this->dictionary, marker_corners, marker_ids);


    std::vector<Vec4i> diamond_ids;
    std::vector<std::vector<cv::Point2f>> diamond_corners;

    // TODO: feed reprojection matrix
    cv::aruco::detectCharucoDiamond(_data, marker_corners[i], ids[i*4],
        this->last_input_frame.diamond_rate,
        diamond_corners, diamond_ids);

    for (size_t i = 0; i < diamond_ids.size(); ++i)
      this->last_input_frame.diamonds[diamond_ids[i]] = diamond_corners[i];
  }

  protected: virtual ignition::math::Pose3d estimatePositionSingle(std::pair<cv::Vec4i, ignition::math::Pose3d> _feature) override
  {
    auto diamond_corners = this->last_input_frame.diamonds.at(_feature.first);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(diamond_corners, this->last_input_frame.square_size,
        prj_matrix, dist_coeffs, rvecs, tvecs);

    auto camera_pose_marker_frame = this->convertFrame(tvecs[0], rvecs[0]);

    return (camera_pose_marker_frame + _feature.second);
  }

  private: ignition::math::Pose3d convertFrame(cv::Vec3d _pos, cv::Vec3d _rot)
  {
    using namespace ignition::math;

    Vector3d position(_pos[0], _pos[1], _pos[2]);
    Vector3d rodrigues(_rot[0], _rot[1], _rot[2]);
    float angle = rodrigues.Length();
    auto axis = rodrigues.Normalize();

    return Pose3d(-position, Quaterniond(axis, 0))+
           Pose3d(Vector3d(0, 0, 0), Quaterniond(axis, -angle));
  }

  protected: cv::aruco::Dictionary dictionary;

  private: LastFrameData last_input_frame;
};
#pragma once

#include <ignition/math.hh>

namespace helper
{
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

  template <class T>
  ignition::math::Quaterniond avgOrientation(T const &collection)
  {
    ignition::math::Quaterniond q_avg;

    double weight_sum_partial = 0;

    for (auto const &item : collection)
    {
      double iweight = weight_sum_partial/(weight_sum_partial+1);

      if (!(iweight < 1 && iweight > 0) && (weight_sum_partial > 0))
        assert(iweight < 1 && iweight > 0);

      weight_sum_partial += 1;

      q_avg = ignition::math::Quaterniond::Slerp(1-iweight, q_avg, item);
    }

    return q_avg;
  }
}

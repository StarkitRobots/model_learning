#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rhoban_model_learning
{
class ArucoTag
{
public:
  ArucoTag();

  ArucoTag(int marker_id, double marker_size, const Eigen::Vector3d& marker_center,
           const Eigen::Quaterniond& orientation);

  /// The aruco identifier
  int marker_id;

  /// Size of the marker [m]
  double marker_size;

  /// Center of the marker [m]
  Eigen::Vector3d marker_center;

  /// Orientation of the marker:
  /// x -> 'right' of the marker
  /// y -> 'bottom' of the marker
  /// 'right' and 'bottom' are defined with respect to the text orientation
  Eigen::Quaterniond orientation;
};

}  // namespace rhoban_model_learning

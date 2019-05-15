#pragma once

#include "starkit_model_learning/model.h"

#include <Eigen/Geometry>

namespace starkit_model_learning
{
/// This model stocks the position and the orientation of an object in a 3d
/// world
class PoseModel : public Model
{
public:
  PoseModel();
  PoseModel(const PoseModel& other);

  /// Return the position inside pose of a point in World referential
  Eigen::Vector3d getPosInSelf(const Eigen::Vector3d& pos_in_world) const;
  /// Return the world position from a position inside pose referential
  Eigen::Vector3d getPosFromSelf(const Eigen::Vector3d& pos_in_self) const;
  /// Return the world to self rotation matrix
  Eigen::Matrix<double, 3, 3> getRotationToSelf() const;
  /// Return the self to world rotation matrix
  Eigen::Matrix<double, 3, 3> getRotationFromSelf() const;

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const override;

  virtual std::unique_ptr<Model> clone() const override;

  /// Position of the object inside the world referential
  Eigen::Vector3d pos;

  /// Orientation of the object
  Eigen::Quaterniond orientation;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace starkit_model_learning

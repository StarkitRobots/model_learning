#pragma once

#include "starkit_model_learning/composite_model.h"
#include "starkit_model_learning/humanoid_models/multi_poses_model.h"

#include <Model/CameraModel.hpp>
#include <Eigen/Core>

namespace starkit_model_learning
{
class InferedPosesModel : public CompositeModel
{
public:
  InferedPosesModel();
  InferedPosesModel(const InferedPosesModel& other);

  double getPxStddev() const;

  const Leph::CameraModel& getCameraModel() const;

  virtual std::unique_ptr<Model> clone() const;

  Eigen::Vector3d getTagPosition(int i) const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;
};

}  // namespace starkit_model_learning

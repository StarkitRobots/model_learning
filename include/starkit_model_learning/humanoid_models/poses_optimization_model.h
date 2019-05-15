#pragma once

#include "starkit_model_learning/composite_model.h"
#include "starkit_model_learning/humanoid_models/multi_poses_model.h"

#include <Model/CameraModel.hpp>

namespace starkit_model_learning
{
class PosesOptimizationModel : public CompositeModel
{
public:
  PosesOptimizationModel();
  PosesOptimizationModel(const PosesOptimizationModel& other);

  double getPxStddev() const;

  const Leph::CameraModel& getCameraModel() const;

  const PoseModel& getPose(int idx) const;

  const Eigen::Vector3d& getTagPosition(int tag_idx) const;

  virtual std::unique_ptr<Model> clone() const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;
};

}  // namespace starkit_model_learning

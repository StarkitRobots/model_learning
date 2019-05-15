#pragma once

#include "starkit_model_learning/model.h"
#include "starkit_model_learning/humanoid_models/pose_model.h"

namespace starkit_model_learning
{
/// This model stocks poses associated to integers. The number of parameters
/// grows linearly with the number of poses.
class MultiPosesModel : public Model
{
public:
  MultiPosesModel();
  MultiPosesModel(const MultiPosesModel& other);

  const PoseModel& getPose(int index) const;

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;

  virtual std::unique_ptr<Model> clone() const;

private:
  /// Poses for each element
  std::vector<PoseModel> poses;
};

}  // namespace starkit_model_learning

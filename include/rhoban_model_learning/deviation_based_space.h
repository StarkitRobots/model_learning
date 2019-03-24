#pragma once

#include "rhoban_model_learning/model_space.h"

#include <Eigen/Core>

namespace rhoban_model_learning
{
/// DeviationBasedSpace is an hyperrectangle centered on the prior and with a
/// size depending on the prior
class DeviationBasedSpace : public ModelSpace
{
public:
  DeviationBasedSpace();

  Eigen::MatrixXd getParametersSpace(const Model& m, const ModelPrior& prior) const override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

private:
  /// space is : mean - ratio * dev to mean + ratio * dev
  double ratio;
};

}  // namespace rhoban_model_learning

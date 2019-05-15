#include "starkit_model_learning/default_prior.h"

namespace starkit_model_learning
{
DefaultPrior::DefaultPrior()
{
}

Eigen::VectorXd DefaultPrior::getParametersMeans(const Model& m) const
{
  (void)m;
  return means;
}

Eigen::VectorXd DefaultPrior::getParametersStdDev(const Model& m) const
{
  (void)m;
  return deviations;
}

std::string DefaultPrior::getClassName() const
{
  return "DefaultPrior";
}

void DefaultPrior::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  means = starkit_utils::readEigen<-1, 1>(json_value, "means");
  deviations = starkit_utils::readEigen<-1, 1>(json_value, "deviations");
}

Json::Value DefaultPrior::toJson() const
{
  Json::Value v;
  v["means"] = starkit_utils::vector2Json(means);
  v["deviations"] = starkit_utils::vector2Json(deviations);
  return v;
}

}  // namespace starkit_model_learning

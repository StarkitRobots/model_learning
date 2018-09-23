#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

namespace rhoban_model_learning
{

VisionNoiseModel::VisionNoiseModel() : px_stddev(1)
{
}

VisionNoiseModel::VisionNoiseModel(const VisionNoiseModel & other)
  : px_stddev(other.px_stddev)
{
}

int VisionNoiseModel::getParametersSize() const {
  return 1;
}

Eigen::VectorXd VisionNoiseModel::getParameters() const {
  Eigen::VectorXd params(1);
  params(0) = px_stddev;
  return params;
}

void VisionNoiseModel::setParameters(const Eigen::VectorXd & new_params) {
  px_stddev = new_params(0);
}

std::vector<std::string> VisionNoiseModel::getParametersNames() const {
  return { "pxStdDev" };
}

void VisionNoiseModel::fromJson(const Json::Value & v,
                                const std::string & dir_name) {
  (void)dir_name;
  rhoban_utils::tryRead(v, "px_stddev", &px_stddev );
}

Json::Value VisionNoiseModel::toJson() const {
  Json::Value v;
  v["px_stddev"] = px_stddev ;
  return v;
}

std::string VisionNoiseModel::getClassName() const {
  return "VisionNoiseModel";
}
  
std::unique_ptr<Model> VisionNoiseModel::clone() const {
  return std::unique_ptr<Model>(new VisionNoiseModel(*this));
}

}

#include "rhoban_model_learning/humanoid_models/rotation_model.h"

namespace rhoban_model_learning
{

RotationModel::RotationModel() : roll(0), pitch(0), yaw(0)
{
}

RotationModel::RotationModel(const RotationModel & other)
  : roll(other.roll), pitch(other.pitch), yaw(other.yaw)
{
}

int RotationModel::getParametersSize() const {
  return 3;
}

Eigen::VectorXd RotationModel::getParameters() const {
  Eigen::VectorXd params(3);
  int idx = 0;
  params(idx++) = roll;
  params(idx++) = pitch;
  params(idx++) = yaw;
  return params;
}

void RotationModel::setParameters(const Eigen::VectorXd & new_params) {
  roll = new_params(0);
  pitch = new_params(1);
  yaw = new_params(2);
}

std::vector<std::string> RotationModel::getParametersNames() const {
  return { "roll", "pitch", "yaw" };
}

void RotationModel::fromJson(const Json::Value & v,
                             const std::string & dir_name) {
  (void)dir_name;
  rhoban_utils::tryRead(v, "roll" , &roll );
  rhoban_utils::tryRead(v, "pitch", &pitch);
  rhoban_utils::tryRead(v, "yaw"  , &yaw  );
}

Json::Value RotationModel::toJson() const {
  Json::Value v;
  v["roll" ] = roll ;
  v["pitch"] = pitch;
  v["yaw"  ] = yaw  ;
  return v;
}

std::string RotationModel::getClassName() const {
  return "RotationModel";
}
  
std::unique_ptr<Model> RotationModel::clone() const {
  return std::unique_ptr<Model>(new RotationModel(*this));
}

}

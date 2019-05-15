#include "starkit_model_learning/humanoid_models/rotation_model.h"

namespace starkit_model_learning
{
RotationModel::RotationModel() : roll(0), pitch(0), yaw(0)
{
}

RotationModel::RotationModel(const RotationModel& other) : roll(other.roll), pitch(other.pitch), yaw(other.yaw)
{
}

Eigen::Vector3d RotationModel::getRPY() const
{
  return Eigen::Vector3d(roll, pitch, yaw);
}

int RotationModel::getParametersSize() const
{
  return 3;
}

Eigen::VectorXd RotationModel::getParameters() const
{
  return getRPY();
}

void RotationModel::setParameters(const Eigen::VectorXd& new_params)
{
  roll = new_params(0);
  pitch = new_params(1);
  yaw = new_params(2);
}

std::vector<std::string> RotationModel::getParametersNames() const
{
  return { "roll", "pitch", "yaw" };
}

void RotationModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryRead(v, "roll", &roll);
  starkit_utils::tryRead(v, "pitch", &pitch);
  starkit_utils::tryRead(v, "yaw", &yaw);
}

Json::Value RotationModel::toJson() const
{
  Json::Value v;
  v["roll"] = roll;
  v["pitch"] = pitch;
  v["yaw"] = yaw;
  return v;
}

std::string RotationModel::getClassName() const
{
  return "RotationModel";
}

std::unique_ptr<Model> RotationModel::clone() const
{
  return std::unique_ptr<Model>(new RotationModel(*this));
}

}  // namespace starkit_model_learning

#include "starkit_model_learning/humanoid_models/pose_model.h"

#include <starkit_utils/util.h>

namespace starkit_model_learning
{
PoseModel::PoseModel()
  : Model(), pos(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond(Eigen::Vector4d(0, 0, 0, 1)))
{
}

PoseModel::PoseModel(const PoseModel& other) : Model(other), pos(other.pos), orientation(other.orientation)
{
}

Eigen::Vector3d PoseModel::getPosInSelf(const Eigen::Vector3d& pos_in_world) const
{
  Eigen::Vector3d vec_in_world = pos_in_world - pos;
  return getRotationToSelf() * vec_in_world;
}

Eigen::Vector3d PoseModel::getPosFromSelf(const Eigen::Vector3d& pos_in_self) const
{
  return pos + getRotationFromSelf() * pos_in_self;
}

Eigen::Matrix<double, 3, 3> PoseModel::getRotationFromSelf() const
{
  return orientation.toRotationMatrix();
}

Eigen::Matrix<double, 3, 3> PoseModel::getRotationToSelf() const
{
  return orientation.toRotationMatrix().transpose();
}

int PoseModel::getParametersSize() const
{
  return 7;
}

Eigen::VectorXd PoseModel::getParameters() const
{
  Eigen::VectorXd parameters(7);
  parameters.segment(0, 3) = pos;
  parameters.segment(3, 4) = orientation.coeffs();
  return parameters;
}

void PoseModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != 7)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid size for new_params, expecting 7, got " +
                             std::to_string(new_params.rows()));
  }
  pos = new_params.segment(0, 3);
  orientation = Eigen::Quaterniond(Eigen::Vector4d(new_params.segment(3, 4)));
  orientation.normalize();
}

std::vector<std::string> PoseModel::getParametersNames() const
{
  return { "x", "y", "z", "qx", "qy", "qz", "qw" };
}

Json::Value PoseModel::toJson() const
{
  Json::Value v;
  v["pos"] = starkit_utils::vector2Json(pos);
  v["orientation"] = starkit_utils::vector2Json(orientation.coeffs());
  return v;
}

void PoseModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  Eigen::Vector4d orientation_tmp;
  starkit_utils::tryReadEigen(v, "pos", &pos);
  if (v.isObject() && v.isMember("orientation"))
  {
    Eigen::Vector4d orientation_tmp = starkit_utils::readEigen<4, 1>(v, "orientation");
    orientation = Eigen::Quaterniond(orientation_tmp);
    orientation.normalize();
  }
}

std::string PoseModel::getClassName() const
{
  return "PoseModel";
}

std::unique_ptr<Model> PoseModel::clone() const
{
  return std::unique_ptr<Model>(new PoseModel(*this));
}

}  // namespace starkit_model_learning

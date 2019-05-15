#include "starkit_model_learning/basic_models/linear_model.h"

#include "starkit_utils/util.h"

namespace starkit_model_learning
{
/// Simple example class with an affine model and a measurement noise
LinearModel::LinearModel() : LinearModel(0)
{
}
LinearModel::LinearModel(int dim) : Model(), coeffs(dim), bias(0.0), std_dev(1.0)
{
}
LinearModel::LinearModel(const LinearModel& other)
  : Model(other), coeffs(other.coeffs), bias(other.bias), std_dev(other.std_dev)
{
}

int LinearModel::getParametersSize() const
{
  return coeffs.rows() + 2;
}

Eigen::VectorXd LinearModel::getParameters() const
{
  Eigen::VectorXd params(getParametersSize());
  int idx = 0;
  params.segment(0, coeffs.rows()) = coeffs;
  idx += coeffs.rows();
  params(idx++) = bias;
  params(idx++) = std_dev;
  return params;
}

void LinearModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != getParametersSize())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid number of parameters (" + std::to_string(new_params.rows()) +
                             ") while expecting " + std::to_string(getParametersSize()));
  }
  if (new_params(new_params.rows() - 1) <= 0)
  {
    throw std::invalid_argument(DEBUG_INFO + "std_dev invalid value: " + std::to_string(std_dev));
  }
  int idx = 0;
  coeffs = new_params.segment(0, coeffs.rows());
  idx += coeffs.rows();
  bias = new_params(idx++);
  std_dev = new_params(idx++);
}

std::unique_ptr<Model> LinearModel::clone() const
{
  return std::unique_ptr<Model>(new LinearModel(*this));
}

Json::Value LinearModel::toJson() const
{
  Json::Value v;
  v["coeffs"] = starkit_utils::vector2Json(coeffs);
  v["bias"] = bias;
  v["std_dev"] = std_dev;
  return v;
}

void LinearModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryReadEigen(v, "coeffs", &coeffs);
  starkit_utils::tryRead(v, "bias", &bias);
  starkit_utils::tryRead(v, "std_dev", &std_dev);
}

std::string LinearModel::getClassName() const
{
  return "LinearModel";
}

}  // namespace starkit_model_learning

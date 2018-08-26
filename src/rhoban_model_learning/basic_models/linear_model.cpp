#include "rhoban_model_learning/basic_models/linear_model.h"

#include "rhoban_model_learning/default_input.h"

#include "rhoban_random/gaussian_distribution.h"
#include "rhoban_utils/util.h"

namespace rhoban_model_learning {

/// Simple example class with an affine model and a measurement noise
LinearModel::LinearModel() : LinearModel(0) {}
LinearModel::LinearModel(int dim) : Model(), coeffs(dim), bias(0.0), std_dev(1.0) {}
LinearModel::LinearModel(const LinearModel & other)
  : Model(other), coeffs(other.coeffs), bias(other.bias), std_dev(other.std_dev) {}


int LinearModel::getParametersSize() const {
  return coeffs.rows() + 2;
}

Eigen::VectorXd LinearModel::getParameters() const {
  Eigen::VectorXd params(getParametersSize());
  int idx = 0;
  params.segment(0, coeffs.rows()) = coeffs; idx += coeffs.rows();
  params(idx++) = bias;
  params(idx++) = std_dev;
  return params;
}

void LinearModel::setParameters(const Eigen::VectorXd & new_params) {
  if (new_params.rows() != getParametersSize()) {
    throw std::runtime_error(DEBUG_INFO + " invalid number of parameters ("
                             + std::to_string(new_params.rows()) + ") while expecting "
                             + std::to_string(getParametersSize()));
  }
  if (new_params(new_params.rows()-1) <= 0) {
    throw std::invalid_argument(DEBUG_INFO + "std_dev invalid value: " + std::to_string(std_dev));
  }
  int idx = 0;
  coeffs = new_params.segment(0, coeffs.rows()); idx += coeffs.rows();
  bias = new_params(idx++);
  std_dev = new_params(idx++);
}
  
Eigen::VectorXd LinearModel::predictObservation(const Input & input,
                                                std::default_random_engine * engine) const {
  double value;
  try {
    Eigen::VectorXd input_vec = (dynamic_cast<const DefaultInput &>(input)).data;
    value = coeffs.transpose() * input_vec + bias;
  } catch(const std::bad_cast & exc) {
    throw std::runtime_error(DEBUG_INFO + "Invalid type for input, expecting DefaultInput");
  }
  if (engine) {
    std::normal_distribution<double> obs_noise_distrib(0, std_dev);
    value += obs_noise_distrib(*engine);
  }
  Eigen::VectorXd result(1);
  result(0) = value;
  return result;
}
  
double LinearModel::computeLogLikelihood(const Sample & sample,
                                         std::default_random_engine * engine) const {
  // Here it is easy to compute the logLikelihood, thus we do not need to use
  // Monte-Carlo
  (void) engine;
  if (sample.getObservation().rows() != 1) {
    throw std::runtime_error(DEBUG_INFO + "Invalid dimension for observation");
  }
  double expected_value = predictObservation(sample.getInput(), nullptr)(0);
  rhoban_random::GaussianDistribution distrib(expected_value, std_dev * std_dev);
  return distrib.getLogLikelihood(sample.getObservation()(0));
}

std::unique_ptr<Model> LinearModel::clone() const {
  return std::unique_ptr<Model>(new LinearModel(*this));
}

Json::Value LinearModel::toJson() const {
  Json::Value v;
  v["coeffs"] = rhoban_utils::vector2Json(coeffs);
  v["bias"] = bias;
  v["std_dev"] = std_dev;
  return v;
}

void LinearModel::fromJson(const Json::Value & v, const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryReadEigen(v, "coeffs", &coeffs);
  rhoban_utils::tryRead(v, "bias", &bias);
  rhoban_utils::tryRead(v, "std_dev", &std_dev);
}

std::string LinearModel::getClassName() const {
  return "LinearModel";
}

}

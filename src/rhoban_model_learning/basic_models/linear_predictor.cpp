#include "rhoban_model_learning/basic_models/linear_predictor.h"

#include "rhoban_model_learning/basic_models/linear_model.h"

#include "rhoban_model_learning/default_input.h"

#include "rhoban_random/gaussian_distribution.h"
#include "rhoban_utils/util.h"

namespace rhoban_model_learning {

LinearPredictor::LinearPredictor() {}
  
Eigen::VectorXd LinearPredictor::predictObservation(const Input & input,
                                                    const Model & model,
                                                    std::default_random_engine * engine) const {
  double value, std_dev;
  Eigen::VectorXd input_vec;
  try {
    input_vec = (dynamic_cast<const DefaultInput &>(input)).data;
  } catch(const std::bad_cast & exc) {
    throw std::runtime_error(DEBUG_INFO + "Invalid type for input, expecting DefaultInput");
  }
  try {
    const LinearModel & linear_model = dynamic_cast<const LinearModel &>(model);
    value = linear_model.coeffs.transpose() * input_vec + linear_model.bias;
  } catch(const std::bad_cast & exc) {
    throw std::runtime_error(DEBUG_INFO + "Invalid type for model, expecting LinearModel");
  }
  if (engine) {
    std::normal_distribution<double> obs_noise_distrib(0, std_dev);
    value += obs_noise_distrib(*engine);
  }
  Eigen::VectorXd result(1);
  result(0) = value;
  return result;
}
  
double LinearPredictor::computeLogLikelihood(const Sample & sample,
                                             const Model & model,
                                             std::default_random_engine * engine) const {
  // Here it is easy to compute the logLikelihood, thus we do not need to use
  // Monte-Carlo
  (void) engine;
  if (sample.getObservation().rows() != 1) {
    throw std::runtime_error(DEBUG_INFO + "Invalid dimension for observation");
  }
  double std_dev;
  try {
    std_dev= dynamic_cast<const LinearModel &>(model).std_dev;
  } catch(const std::bad_cast & exc) {
    throw std::runtime_error(DEBUG_INFO + "Invalid type for model, expecting LinearModel");
  }
  
  double expected_value = predictObservation(sample.getInput(), model, nullptr)(0);
  rhoban_random::GaussianDistribution distrib(expected_value, std_dev * std_dev);
  return distrib.getLogLikelihood(sample.getObservation()(0));
}

std::string LinearPredictor::getClassName() const {
  return "LinearPredictor";
}

}

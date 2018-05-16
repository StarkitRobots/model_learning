#include "rhoban_model_learning/ball_models/linear_speed_estimator.h"

#include "rhoban_model_learning/ball_models/position_sequence.h"

#include "rhoban_utils/util.h"

namespace rhoban_model_learning
{

LinearSpeedEstimator::LinearSpeedEstimator()
  : SpeedEstimator(1), window_duration(0.5)
{
}

LinearSpeedEstimator::~LinearSpeedEstimator() {}

Eigen::VectorXd
LinearSpeedEstimator::predictObservation(const Input & input,
                                         std::default_random_engine * engine) const {
  try {
    const PositionSequence & seq = dynamic_cast<const PositionSequence &>(input);

    //TODO: treat sequence
    // 1. Remove data outside of time window
    // 2. Choose estimation_point
    // 3. Separate data in two cluster
    // 4. Average position in each cluster
    // 5. Use positions and dt to compute average speed
    throw std::logic_error(DEBUG_INFO + " not implemented");
  } catch (const std::bad_cast & exc) {
    throw std::logic_error(DEBUG_INFO + " invalid type for input");    
  }
}

Eigen::VectorXd LinearSpeedEstimator::getGlobalParameters() const {
  Eigen::VectorXd params(1);
  params(0) = window_duration;
  return params;
}

Eigen::MatrixXd LinearSpeedEstimator::getGlobalParametersSpace() const {
  Eigen::MatrixXd limits(1,2);
  limits(0,0) = 0.01;
  limits(0,1) = 1.0;
  return limits;
}

void LinearSpeedEstimator::setGlobalParameters(const Eigen::VectorXd & new_params) {
  if (new_params.rows() != 1) {
    throw std::logic_error(DEBUG_INFO + " invalid number of parameters, "
                           + std::to_string(new_params.rows()) + " received, 1 expected");
  }
  window_duration = new_params(0);
}

std::vector<std::string> LinearSpeedEstimator::getGlobalParametersNames() const {
  return { "window_duration"};
}

Json::Value LinearSpeedEstimator::toJson() const {
  Json::Value v;
  v["window_duration"] = window_duration;
  return v;
}

void LinearSpeedEstimator::fromJson(const Json::Value & v, const std::string & dir_name) {
  (void)dir_name;
  rhoban_utils::tryRead(v, "window_duration",  &window_duration);
}

std::string LinearSpeedEstimator::getClassName() const {
  return "LinearSpeedEstimator";
}

}

#include "rhoban_model_learning/ball_models/ball_physical_model.h"

namespace rhoban_model_learning
{

LinearSpeedEstimator::LinearSpeedEstimator()
  : ModularModel(1), window_duration(0.5)
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
    // 5. Use positions and dt to compute average position
    throw std::logic_error(DEBUG_INFO + " not implemented");
  } catch (const std::bad_cast & exc) {
    throw std::logic_error(DEBUG_INFO + " invalid type for input");    
  }
}

Eigen::VectorXd LinearSpeedEstimator::GetGlobalParameters() const {
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
                           + new_params.rows() + " received, 1 expected");
  }
  window_duration = new_params(0);
}

std::vector<std::string> LinearSpeedEstimator::getGlobalParametersNames() const {
  return { "window_duration"};
}

Json::Value LinearSpeedEstimator::toJson() const {
  Json::Value v;
  v["blade_grass_direction"] = blade_grass_direction.getSignedValue();
  v["max_integration_step" ] = max_integration_step;
  return v;
}

void LinearSpeedEstimator::fromJson(const Json::Value & v, const std::string & dir_name) {
  double bgd_deg = blade_grass_direction.getSignedValue();
  rhoban_utils::tryRead(v, "blade_grass_direction",  &bdg_deg             );
  rhoban_utils::tryRead(v, "max_integration_step" ,  &max_integration_step);
  blade_grass_direction = Angle(bdg_deg);
}

std::string LinearSpeedEstimator::getClassName() const {
  return "LinearSpeedEstimator";
}

}

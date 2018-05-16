#include "rhoban_model_learning/ball_models/trajectory_predictor.h"

#include "rhoban_model_learning/ball_models/position_predictor_factory.h"
#include "rhoban_model_learning/ball_models/speed_estimator_factory.h"

#include "rhoban_utils/util.h"

namespace rhoban_model_learning
{

TrajectoryPredictor::TrajectoryPredictor()
  : ModularModel(0), speed_estimator(), position_predictor()
{
}

TrajectoryPredictor::~TrajectoryPredictor() {}

Eigen::VectorXd
TrajectoryPredictor::predictObservation(const Input & input,
                                        std::default_random_engine * engine) const {
  throw std::logic_error(DEBUG_INFO + " unimplemented");
}


Eigen::VectorXi TrajectoryPredictor::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(2);
}

Eigen::VectorXd TrajectoryPredictor::getGlobalParameters() const {
  Eigen::VectorXd se_params = speed_estimator->getGlobalParameters();
  Eigen::VectorXd pp_params = position_predictor->getGlobalParameters();
  Eigen::VectorXd params(se_params.rows() + pp_params.rows());
  params << se_params, pp_params;
  return params;
}

Eigen::MatrixXd TrajectoryPredictor::getGlobalParametersSpace() const {
  Eigen::VectorXd se_limits = speed_estimator->getGlobalParametersSpace();
  Eigen::VectorXd pp_limits = position_predictor->getGlobalParametersSpace();
  Eigen::MatrixXd limits(se_limits.rows() + pp_limits.rows(),2);
  limits << se_limits, pp_limits;
  return limits;
}

void TrajectoryPredictor::setGlobalParameters(const Eigen::VectorXd & new_params) {
  int se_rows = speed_estimator->getGlobalParametersCount();
  int pp_rows = position_predictor->getGlobalParametersCount();
  if (new_params.rows() != se_rows + pp_rows) {
    throw std::logic_error(DEBUG_INFO + " Invalid number of parameters: "
                           + std::to_string(new_params.rows()) + " received while expecting "
                           + std::to_string(se_rows + pp_rows));
  }
  speed_estimator->setGlobalParameters(new_params.segment(0,se_rows));
  position_predictor->setGlobalParameters(new_params.segment(se_rows, pp_rows));
}

std::vector<std::string> TrajectoryPredictor::getGlobalParametersNames() const {
  std::vector<std::string> names, se_names, pp_names;
  se_names = speed_estimator->getGlobalParametersNames();
  pp_names = position_predictor->getGlobalParametersNames();
  names.insert(names.end(), se_names.begin(), se_names.end());
  names.insert(names.end(), pp_names.begin(), pp_names.end());
  return names;
}

Json::Value TrajectoryPredictor::toJson() const {
  Json::Value v;
  v["speed_estimator"] = speed_estimator->toFactoryJson();
  v["position_predictor"] = position_predictor->toFactoryJson();
  return v;
}

void TrajectoryPredictor::fromJson(const Json::Value & v, const std::string & dir_name) {
  speed_estimator = SpeedEstimatorFactory().read(v, "speed_estimator", dir_name);
  position_predictor = PositionPredictorFactory().read(v, "position_predictor", dir_name);
}

std::string TrajectoryPredictor::getClassName() const {
  return "TrajectoryPredictor";
}

}

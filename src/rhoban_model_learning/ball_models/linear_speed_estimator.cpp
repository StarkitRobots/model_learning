#include "rhoban_model_learning/ball_models/linear_speed_estimator.h"

#include "rhoban_model_learning/ball_models/position_sequence.h"

#include "rhoban_utils/util.h"

#include <iostream>

namespace rhoban_model_learning
{

LinearSpeedEstimator::LinearSpeedEstimator()
  : SpeedEstimator(1), window_duration(0.5)
{
}

LinearSpeedEstimator::~LinearSpeedEstimator() {}

Eigen::VectorXd
LinearSpeedEstimator::predictObservation(const rhoban_model_learning::Input & raw_input,
                                         std::default_random_engine * engine) const {
  try {
    const SpeedEstimator::Input & input = dynamic_cast<const SpeedEstimator::Input &>(raw_input);

    double start = input.prediction_time - window_duration / 2;
    double end = input.prediction_time + window_duration / 2;

    // 1. Remove data outside of time window
    PositionSequence seq = input.seq.extractSequence(start,end);
    // 2. Separate data in two cluster
    size_t nb_entries = seq.timed_positions.size();
    size_t entries_below = nb_entries / 2;
    size_t entries_above = nb_entries - entries_below;
    if (nb_entries < 2) {
      std::cout << "Less than 2 entries in [" << start << "," << end << "]" << std::endl;
      return Eigen::Vector2d(0,0);
    }
    // 3. Average position in each cluster
    Eigen::Vector3d part1_avg = Eigen::Vector3d::Zero();
    Eigen::Vector3d part2_avg = Eigen::Vector3d::Zero();
    for (size_t idx = 0; idx < entries_below; idx++) {
      part1_avg += seq.timed_positions[idx];
    }
    part1_avg /= entries_below;
    for (size_t idx = entries_below; idx < nb_entries; idx++) {
      part2_avg += seq.timed_positions[idx];
    }
    part2_avg /= entries_above;
    // 4. Use positions and dt to compute average speed
    Eigen::Vector2d distance = part2_avg.segment(1,2) - part1_avg.segment(1,2);
    double dt = part2_avg(0) - part1_avg(0);
    Eigen::Vector2d ball_speed = distance / dt;
    // 5. Adding simple noise
    // TODO: replace by a parameter and take into account dispersion of data
    if (engine != nullptr) {
      std::uniform_real_distribution<double> noise_distrib(-0.1,0.1);
      ball_speed(0) += noise_distrib(*engine);
      ball_speed(1) += noise_distrib(*engine);
    }
    return ball_speed;
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
  Json::Value v = ModularModel::toJson();
  v["window_duration"] = window_duration;
  return v;
}

void LinearSpeedEstimator::fromJson(const Json::Value & v, const std::string & dir_name) {
  ModularModel::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "window_duration",  &window_duration);
}

std::string LinearSpeedEstimator::getClassName() const {
  return "LinearSpeedEstimator";
}

}

#include "starkit_model_learning/ball_models/speed_estimator.h"

namespace starkit_model_learning
{
SpeedEstimator::Input::Input()
{
}
SpeedEstimator::Input::Input(const Input& other) : seq(other.seq), prediction_time(other.prediction_time)
{
}

SpeedEstimator::Input::~Input()
{
}

std::unique_ptr<starkit_model_learning::Input> SpeedEstimator::Input::clone() const
{
  return std::unique_ptr<starkit_model_learning::Input>(new Input(*this));
}

SpeedEstimator::SpeedEstimator() : Model()
{
}

Eigen::VectorXi SpeedEstimator::getObservationsCircularity() const
{
  return Eigen::VectorXi::Zero(2);
}

}  // namespace starkit_model_learning

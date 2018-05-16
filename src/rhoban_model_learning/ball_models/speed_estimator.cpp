#include "rhoban_model_learning/ball_models/speed_estimator.h"

namespace rhoban_model_learning
{

SpeedEstimator::SpeedEstimator(int nb_dims) : ModularModel(nb_dims) {}

Eigen::VectorXi SpeedEstimator::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(3);
}

}

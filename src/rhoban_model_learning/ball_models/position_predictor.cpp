#include "rhoban_model_learning/ball_models/position_predictor.h"

namespace rhoban_model_learning
{

PositionPredictor::PositionPredictor(int nb_dims) : ModularModel(nb_dims) {}

Eigen::VectorXi PositionPredictor::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(4);
}

}

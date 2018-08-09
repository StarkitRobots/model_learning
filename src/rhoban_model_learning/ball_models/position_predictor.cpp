#include "rhoban_model_learning/ball_models/position_predictor.h"

namespace rhoban_model_learning
{
PositionPredictor::Input::Input() {}
PositionPredictor::Input::Input(const Input & other)
  : ball_pos(other.ball_pos), ball_speed(other.ball_speed),
    prediction_duration(other.prediction_duration)
{}

std::unique_ptr<rhoban_model_learning::Input> PositionPredictor::Input::clone() const
{
  return std::unique_ptr<rhoban_model_learning::Input>(new Input(*this));
}

PositionPredictor::PositionPredictor(int nb_dims) : Model() {}

Eigen::VectorXi PositionPredictor::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(4);
}

}

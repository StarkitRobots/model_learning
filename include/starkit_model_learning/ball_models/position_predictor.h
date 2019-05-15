#pragma once

#include "starkit_model_learning/model.h"

namespace starkit_model_learning
{
/// A position predictor estimates the position of the ball after a specified
/// amount of time
///
/// It takes as input the position and the speed of the ball as well as the time
/// requested for prediction (dt)
class PositionPredictor : public Model
{
public:
  class Input : public starkit_model_learning::Input
  {
  public:
    Input();
    Input(const Input& other);

    std::unique_ptr<starkit_model_learning::Input> clone() const override;

    /// The current mesured position of the ball [m]
    Eigen::Vector2d ball_pos;
    /// The current measured speed of the ball [m/s]
    Eigen::Vector2d ball_speed;
    /// The expected time before prediction [s]
    double prediction_duration;
  };

  PositionPredictor(int dim);

  virtual Eigen::VectorXi getObservationsCircularity() const override;
};

}  // namespace starkit_model_learning

#pragma once

#include "rhoban_model_learning/modular_model.h"

namespace rhoban_model_learning
{

/// A position predictor estimates the position of the ball after a specified
/// amount of time
///
/// It takes as input the position and the speed of the ball as well as the time
/// requested for prediction (dt)
class PositionPredictor : public ModularModel {
public:
  class Input : public rhoban_model_learning::Input {
  public:
    /// The current mesured position of the ball [m]
    Eigen::Vector2d ball_pos;
    /// The current measured speed of the ball [m/s]
    Eigen::Vector2d ball_speed;
    /// The expected time before prediction [s]
    double prediction_duration;
  };

  
  PositionPredictor(int dim);

  /// Input: a PositionPredictorInput
  /// @return:  (ball_x [m], ball_y [m], vel_x [m/s], vel_y [m/s])
  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const = 0;


  virtual Eigen::VectorXi getObservationsCircularity() const override;

};

}

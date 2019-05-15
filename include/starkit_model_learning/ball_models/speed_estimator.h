#pragma once

#include "starkit_model_learning/model.h"

#include "starkit_model_learning/ball_models/position_sequence.h"

namespace starkit_model_learning
{
/// Speed estimator estimates the speed of the ball from a sequence of positions
///
/// It takes as input a sequence of timed ball position (@see PositionSequence)
/// and the time for the prediction
///
/// It produces an output with the estimated speed of the ball (vx, vy) [m/s]
class SpeedEstimator : public Model
{
public:
  class Input : public starkit_model_learning::Input
  {
  public:
    Input();
    Input(const Input& other);
    ~Input();

    std::unique_ptr<starkit_model_learning::Input> clone() const override;

    PositionSequence seq;
    double prediction_time;
  };

  SpeedEstimator();

  /// Input: a PositionSequence is required
  /// @return:  (vx [m/s], vy [m/s], t_pred [s])
  virtual Eigen::VectorXd predictObservation(const starkit_model_learning::Input& input,
                                             std::default_random_engine* engine) const = 0;

  virtual Eigen::VectorXi getObservationsCircularity() const override;
};

}  // namespace starkit_model_learning

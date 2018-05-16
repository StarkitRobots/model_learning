#pragma once

#include "rhoban_model_learning/modular_model.h"

namespace rhoban_model_learning
{

/// Speed estimator estimates the speed of the ball from a sequence of positions
///
/// It takes as input a sequence of timed ball position (@see PositionSequence)
/// It produces an output with the estimated speed of the ball and the corresponding time
/// - (vx, vy, t_pred)
class SpeedEstimator : public ModularModel {
public:
  SpeedEstimator(int nb_dims);

  /// Input: a PositionSequence is required
  /// @return:  (vx [m/s], vy [m/s], t_pred [s])
  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const = 0;
  
  virtual Eigen::VectorXi getObservationsCircularity() const override;
};

}

#pragma once

#include "rhoban_model_learning/input.h"

#include <Eigen/Core>

#include <map>

namespace rhoban_model_learning
{

class PositionSequence : public Input {
public:
  /// Creates an empty position sequence
  PositionSequence();
  PositionSequence(const PositionSequence & other);
  virtual ~PositionSequence();

  virtual std::unique_ptr<Input> clone() const override;


  /// The entries of the position sequence 
  /// Key is timestamp in [s]
  /// Value is position in [m] (referential is free)
  std::map<double,Eigen::Vector2d> timed_positions;
};

}

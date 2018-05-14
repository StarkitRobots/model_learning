#pragma once

#include "rhoban_model_learning/input.h"

namespace rhoban_model_learning
{

class PositionSequence : public Input {
public:
  /// Creates an empty position sequence
  PositionSequence();
  virtual ~PositionSequence();


  /// The entries of the position sequence 
  /// Key is timestamp in [s]
  /// Value is position in [m] (referential is free)
  std::map<double,Eigen::Vector2d> timed_positions;
};

}

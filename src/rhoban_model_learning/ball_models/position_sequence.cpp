#include "rhoban_model_learning/ball_models/position_sequence.h"

namespace rhoban_model_learning
{

PositionSequence::PositionSequence() {}

PositionSequence::PositionSequence(const PositionSequence & other)
  : timed_positions(other.timed_positions)
{
}

PositionSequence::~PositionSequence() {}

std::unique_ptr<Input> PositionSequence::clone() const {
  return std::unique_ptr<Input>(new PositionSequence(*this));
}

}

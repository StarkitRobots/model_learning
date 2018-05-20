#include "rhoban_model_learning/ball_models/position_sequence.h"

#include "rhoban_utils/util.h"

namespace rhoban_model_learning
{

PositionSequence::PositionSequence() {}

PositionSequence::PositionSequence(const PositionSequence & other)
  : timed_positions(other.timed_positions)
{
}

PositionSequence::~PositionSequence() {}

void PositionSequence::addEntry(const Eigen::Vector3d & entry) {
  if (timed_positions.size() > 0 && timed_positions.back()(0) > entry(0)) {
    throw std::logic_error(DEBUG_INFO + " incompatible timing for adding entry");
  }
  timed_positions.push_back(entry);
}

PositionSequence PositionSequence::extractSequence(double start, double end) const {
  PositionSequence subsequence;
  for (const Eigen::Vector3d & entry : timed_positions) {
    double t = entry(0);
    if (t >= start && t <= end) {
      subsequence.addEntry(entry);
    }
  }
  return subsequence;
}

double PositionSequence::getStart() const {
  return timed_positions.front()(0);
}

double PositionSequence::getEnd() const {
  return timed_positions.back()(0);
}

std::unique_ptr<Input> PositionSequence::clone() const {
  return std::unique_ptr<Input>(new PositionSequence(*this));
}

}

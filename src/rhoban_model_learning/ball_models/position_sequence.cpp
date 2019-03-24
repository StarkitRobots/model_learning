#include "rhoban_model_learning/ball_models/position_sequence.h"

#include "rhoban_utils/util.h"

namespace rhoban_model_learning
{
PositionSequence::PositionSequence()
{
}

PositionSequence::PositionSequence(const PositionSequence& other) : timed_positions(other.timed_positions)
{
}

PositionSequence::~PositionSequence()
{
}

bool PositionSequence::isEmpty() const
{
  return timed_positions.size() == 0;
}

void PositionSequence::addEntry(const Eigen::Vector3d& entry)
{
  if (timed_positions.size() > 0 && timed_positions.back()(0) > entry(0))
  {
    throw std::logic_error(DEBUG_INFO + " incompatible timing for adding entry");
  }
  timed_positions.push_back(entry);
}

PositionSequence PositionSequence::extractSequence(double start, double end) const
{
  PositionSequence subsequence;
  for (const Eigen::Vector3d& entry : timed_positions)
  {
    double t = entry(0);
    if (t >= start && t <= end)
    {
      subsequence.addEntry(entry);
    }
  }
  return subsequence;
}

Eigen::Vector3d PositionSequence::getStart() const
{
  if (isEmpty())
  {
    throw std::logic_error(DEBUG_INFO + "empty sequence");
  }
  return timed_positions.front();
}

Eigen::Vector3d PositionSequence::getEnd() const
{
  if (isEmpty())
  {
    throw std::logic_error(DEBUG_INFO + "empty sequence");
  }
  return timed_positions.back();
}

Eigen::Vector2d PositionSequence::getStartPos() const
{
  return getStart().segment(1, 2);
}

Eigen::Vector2d PositionSequence::getEndPos() const
{
  return getEnd().segment(1, 2);
}

double PositionSequence::getStartTime() const
{
  return getStart()(0);
}

double PositionSequence::getEndTime() const
{
  return getEnd()(0);
}

std::unique_ptr<Input> PositionSequence::clone() const
{
  return std::unique_ptr<Input>(new PositionSequence(*this));
}

size_t PositionSequence::size() const
{
  return timed_positions.size();
}

}  // namespace rhoban_model_learning

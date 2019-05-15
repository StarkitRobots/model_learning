#pragma once

#include "starkit_model_learning/input.h"

#include <Eigen/Core>

#include <vector>

namespace starkit_model_learning
{
class PositionSequence : public Input
{
public:
  /// Creates an empty position sequence
  PositionSequence();
  PositionSequence(const PositionSequence& other);
  virtual ~PositionSequence();

  bool isEmpty() const;

  /// throws an exception if timing of the entry is lower than timing of the
  /// last entry
  void addEntry(const Eigen::Vector3d& entry);

  /// Extract a position sequence with only the entries in the given interval
  PositionSequence extractSequence(double start, double end) const;

  /// Return the timestamp of the first entry
  double getStartTime() const;

  /// Return the timestamp of the latest entry
  double getEndTime() const;

  /// Return the position of the first entry
  Eigen::Vector2d getStartPos() const;

  /// Return the position of the latest entry
  Eigen::Vector2d getEndPos() const;

  /// Return the first entry
  Eigen::Vector3d getStart() const;

  /// Return the latest entry
  Eigen::Vector3d getEnd() const;

  virtual std::unique_ptr<Input> clone() const override;

  size_t size() const;

  /// The entries of the position sequence (ordered by time)
  /// Entries are: (time[s], ball_x[m], ball_y[m])
  std::vector<Eigen::Vector3d> timed_positions;
};

}  // namespace starkit_model_learning

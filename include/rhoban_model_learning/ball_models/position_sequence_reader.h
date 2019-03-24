#pragma once

#include "rhoban_model_learning/ball_models/position_sequence.h"
#include "rhoban_model_learning/ball_models/speed_estimator.h"

namespace rhoban_model_learning
{
/// Extracts a map of PositionSequences using the provided
class PositionSequenceReader : public rhoban_utils::JsonSerializable
{
public:
  PositionSequenceReader();

  /// Read all the ball_position entries at the given path, then separates them
  /// based on the speed provided by the speed_estimator and return a vector
  /// containing the separated position sequences
  std::vector<PositionSequence> readPositionSequences(const std::string& file_path) const;

  /// Separate a PositionSequence in multiple PositionSequences based on
  /// estimated ball speed
  std::vector<PositionSequence> splitSequence(const PositionSequence& pos_seq) const;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// Used to estimate the speed of the ball in order to separate different ball
  /// trajectories
  std::unique_ptr<SpeedEstimator> speed_estimator;

  /// Above this speed, create a new trajectory if authorized [m/s]
  double high_threshold;

  /// Below this speed, authorize the creation of a new trajectory [m/s]
  double low_threshold;

  /// Sequence starts 'anticipation' before the first entry with a speed above
  /// the threshold. This option allows to have some data entries before the
  /// ball actually start moving
  double anticipation;

  /// If time gap between two entries is larger, automatically starts a new sequence [s]
  double allowed_time_gap;

  /// If the sequences' first and last point are at a distance smaller than min_amplitude, ignore the sequence.
  double min_amplitude;

  /// If the sequence as less than min_point_count points, ignore the sequence.
  double min_point_count;
};

}  // namespace rhoban_model_learning

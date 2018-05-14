#pragma once

#include "rhoban_model_learning/position_sequence.h"

namespace rhoban_model_learning
{

///class TrajectoryPredictorInputReader : public InputReader {
///  /// Extracts training and validation set from 
///  DataSet extractSamples(const std::string & file_path,
///                         std::default_random_engine * engine) const;
///};

class TrajectoryPredictorInput : public Input {
public:
  TrajectoryPredictorInput();
  virtual ~TrajectoryPredictorInput();

  /// The entries available for prediction
  PositionSequence ball_positions;

  /// Timestamp for the prediction
  double prediction_time;
};

class TrajectoryPredictor : public ModularModel {

private:
  /// The model used to estimate the speed of the ball
  std::unique_ptr<SpeedEstimator> speed_estimator;

  /// The physical model used to predict ball speed
  std::unique_ptr<PositionPredictor> position_predictor;

}


}

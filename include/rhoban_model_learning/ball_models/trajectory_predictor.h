#pragma once

#include "rhoban_model_learning/ball_models/position_sequence.h"
#include "rhoban_model_learning/ball_models/position_predictor.h"
#include "rhoban_model_learning/ball_models/speed_estimator.h"

#include "rhoban_model_learning/input_reader.h"
#include "rhoban_model_learning/modular_model.h"

namespace rhoban_model_learning
{

class TrajectoryPredictorInputReader : public InputReader {
public:
  /// Extracts training and validation set from 
  DataSet extractSamples(const std::string & file_path,
                         std::default_random_engine * engine) const;

private:
  /// Used to extract Position Sequences
//  PositionSequenceReader sequence_reader;

  /// Minimal time from trajectory start to sequence start [s]
  double min_time_to_start;

  /// Maximal time from trajectory start to sequence start [s]
  double max_time_to_start;

  /// What is the duration of the sequence available for prediction [s]
  double memory_duration;

  /// Min time from last sample to prediction [s]
  double min_dt;

  /// Max time from last sample to prediction [s]
  double max_dt;
};

/// Predicts the position of the ball at a given TimeStamp
class TrajectoryPredictor : public ModularModel {
public:

  class Input : public rhoban_model_learning::Input {
  public:
    Input();
    virtual ~Input();
    
    /// The entries available for prediction
    PositionSequence ball_positions;
    
    /// Timestamp for the prediction
    double prediction_time;
  };


  TrajectoryPredictor();
  virtual ~TrajectoryPredictor();

//  virtual Eigen::VectorXd
//  predictObservation(const Input & input,
//                     std::default_random_engine * engine) const override;
  
  
  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd getGlobalParameters() const override;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const override;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getGlobalParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

private:
  /// The model used to estimate the speed of the ball
  std::unique_ptr<SpeedEstimator> speed_estimator;

  /// The physical model used to predict ball speed
  std::unique_ptr<PositionPredictor> position_predictor;
};


}

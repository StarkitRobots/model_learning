#pragma once

#include "rhoban_model_learning/ball_models/position_sequence.h"
#include "rhoban_model_learning/ball_models/position_predictor.h"
#include "rhoban_model_learning/ball_models/speed_estimator.h"

#include "rhoban_model_learning/input_reader.h"
#include "rhoban_model_learning/modular_model.h"
#include "rhoban_model_learning/ball_models/position_sequence_reader.h"

namespace rhoban_model_learning
{
/// Predicts the position of the ball (x,y) at a given TimeStamp
class TrajectoryPredictor : public ModularModel {
public:

  class Input : public rhoban_model_learning::Input {
  public:
    Input();
    Input(const PositionSequence & seq, double prediction_time);
    Input(const Input & other);
    virtual ~Input();

    std::unique_ptr<rhoban_model_learning::Input> clone() const override;
    
    /// The entries available for prediction
    PositionSequence ball_positions;
    
    /// Timestamp for the prediction
    double prediction_time;
  };

  class Reader : public InputReader {
  public:
    /// Extracts training and validation set from 
    DataSet extractSamples(const std::string & file_path,
                           std::default_random_engine * engine) const;

    Json::Value toJson() const override;
    void fromJson(const Json::Value & v, const std::string & dir_name) override;
    std::string getClassName() const;

  private:
    /// Used to extract Position Sequences
    PositionSequenceReader sequence_reader;

    /// The number of sequences used for training
    int nb_training_sequences;

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



  TrajectoryPredictor();
  TrajectoryPredictor(const TrajectoryPredictor & other);
  virtual ~TrajectoryPredictor();

  Eigen::VectorXd
  predictObservation(const rhoban_model_learning::Input & input,
                     std::default_random_engine * engine) const override;
  
  
  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd getGlobalParameters() const override;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const override;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getGlobalParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

  std::unique_ptr<Model> clone() const override;
private:
  /// The model used to estimate the speed of the ball
  std::unique_ptr<SpeedEstimator> speed_estimator;

  /// The physical model used to predict ball speed
  std::unique_ptr<PositionPredictor> position_predictor;
};


}

#pragma once

#include "rhoban_model_learning/modular_model.h"
#include "rhoban_model_learning/input_reader.h"

#include "Model/HumanoidModel.hpp"
#include "Model/NamesModel.h"

namespace rhoban_model_learning
{
/// Inputs are composed of:
/// - Position of all the DOF and the IMU
/// - Expected position of the data in the left_foot referential
///
/// Observations are positions of the object in the image (x,y)

/// This class is used to learn the model of the ball
///
/// Inputs are composed:
/// - Trajectories of balls in the referential of Todo

class BallPredictionModel : public ModularModel
{
public:
  class BallTrajectoriesInput : public Input
  {
  public:
    BallTrajectoriesInput();
    BallTrajectoriesInput(const Leph::VectorLabel& data);
    BallTrajectoriesInput(const BallTrajectoriesInput& other);

    virtual std::unique_ptr<Input> clone() const override;

    /// Contains all the ball positions for each trajectory
    Leph::VectorLabel data;
  };

  class BallTrajectoriesInputReader : public InputReader
  {
  public:
    BallTrajectoriesInputReader();

    /// If k=3
    /// .csv file : trajectory_id, xR, yR, thetaR, time, xB, yB
    virtual DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

    virtual std::string getClassName() const override;
    Json::Value toJson() const override;
    void fromJson(const Json::Value& v, const std::string& dir_name) override;

  private:
    /// Number of points used to predict next point
    int nb_input_points;
    /// Number of trajectories used for the training set
    int nb_training_trajectories;
    /// Number of trajectories used for the validation set
    int nb_validation_trajectories;
    /// The maximal number of samples per trajectory allowed in training and validation set
    int max_samples_per_trajectories;
    /// The minimal number of samples per trajectory allowed in training and validation set
    int min_samples_per_trajectories;
    /// The maximal time difference between two points
    int max_dt_observation;
    /// The minimal time difference between two points
    int min_dt_observation;
    /// Is the reading process printing summary of reading?
    int max_dt_prediction;
    /// The minimal time difference between two points
    int min_dt_prediction;
    /// Is the reading process printing summary of reading?
    bool verbose;
  };

  BallPredictionModel();
  BallPredictionModel(const BallPredictionModel& other);

  virtual Eigen::VectorXd getGlobalParameters() const override;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const override;
  virtual void setGlobalParameters(const Eigen::VectorXd& new_params) override;
  virtual std::vector<std::string> getGlobalParametersNames() const override;
  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd predictObservation(const Input& input, std::default_random_engine* engine) const;

  /// Density
  virtual double computeLogLikelihood(const Sample& sample, std::default_random_engine* engine) const override;

  virtual std::unique_ptr<Model> clone() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const;

  /// Convert from leph coordinates ([-1,1]^2) to img coordinates ([0,width]*[0,height])
  Eigen::Vector2d leph2Img(const Eigen::Vector2d& leph_coordinates) const;
  /// Convert from img coordinates ([0,width]*[0,height]) to leph coordinates ([-1 1]^2)
  Eigen::Vector2d img2Leph(const Eigen::Vector2d& img_coordinates) const;

private:
  /// Hyper parameters
};

}  // namespace rhoban_model_learning

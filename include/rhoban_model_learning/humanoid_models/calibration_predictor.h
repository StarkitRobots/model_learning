#pragma once

#include "rhoban_model_learning/predictor.h"

namespace rhoban_model_learning
{
/// Inputs should be CalibrationInput which contain:
/// - Position of all the DOF and the IMU
/// - Expected position of the data in the left_foot referential
///
/// Observations are positions of the object in the image (x,y)
class CalibrationPredictor : Predictor
{
public:
  CalibrationPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  Eigen::VectorXi getObservationsCircularity() const override;

  std::string getClassName() const override;
};

}  // namespace rhoban_model_learning

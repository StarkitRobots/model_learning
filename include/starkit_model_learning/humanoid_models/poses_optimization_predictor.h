#pragma once

#include "starkit_model_learning/predictor.h"

namespace starkit_model_learning
{
/// Inputs should be PosesOptimizationInput which contain:
/// - Image number
/// - Aruco Id
///
/// Observations are positions of the marker in the image (x,y)
class PosesOptimizationPredictor : public Predictor
{
public:
  PosesOptimizationPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  Eigen::VectorXi getObservationsCircularity() const override;

  std::string getClassName() const override;
};

}  // namespace starkit_model_learning

#include "starkit_model_learning/predictor.h"

namespace starkit_model_learning
{
/// Predict observations based on
///
/// Expected type for Inputs is DefaultInput
///
/// Note: the predictor could easily be modified to accept multi-dimensional output if required
class LinearPredictor : public Predictor
{
public:
  LinearPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  std::string getClassName() const override;
};

}  // namespace starkit_model_learning

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning {

/// Simple class with an affine model and a measurement noise along the single
/// observation dimension
///
/// Expected type for Inputs is DefaultInput
///
/// Note: the model could easily be modified to accept multi-dimensional output if required
class LinearModel : public Model {
public:
  LinearModel();
  LinearModel(int dim);
  LinearModel(const LinearModel & other);

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd & new_params) override;
  
  Eigen::VectorXd predictObservation(const Input & input,
                                     std::default_random_engine * engine) const override;
  
  double computeLogLikelihood(const Sample & sample,
                              std::default_random_engine * engine) const override;

  std::unique_ptr<Model> clone() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

private:
  /// Coefficients of the affine function
  Eigen::VectorXd coeffs;
  
  /// Value at origin
  double bias;
  
  /// Measurement noise
  double std_dev;
};

}

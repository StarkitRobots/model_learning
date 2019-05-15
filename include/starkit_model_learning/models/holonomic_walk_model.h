#pragma once

#include "starkit_model_learning/model.h"

namespace starkit_model_learning
{
/// This class is used to integrate steps of the robot. It uses as input an
/// estimation of displacement and output a corrected version of the orders
///
/// Possible types of input:
/// - Order provided to the walk engine (Predictive motion model)
/// - Measurement of displacement according to the robot sensors (Odometry model)
///
/// Notation:
/// - X   : denotes the input provided to the model
/// - Y   : denotes the output computed by the model
/// - N(v): denotes a multivariate gaussian distribution with v.rows()
///         independent variables, with v_i the standard deviation along
///         dimension i
///
/// This model is composed two different parts:
/// 1. Deterministic part:
///    deterministic_y = deterministic_prop * X + deterministic_bias
/// 2. Stochastic part:
///    noise_y = N(noise_prop * X + noise_constant)

class HolonomicWalkModel : public Model
{
public:
  /// There are different variations of parametrization for the model
  /// (both deterministic and stochastic):
  /// None        : No parameters can be changed (mainly related to stochastic parameters)
  /// Constant    : Only constant values can be changed
  /// Proportional: Only proportional parameters can be changed
  /// All         : All the parameters are used
  ///
  /// Important note: The choice of the parameter subset only affect which
  ///                 parameters can be manipulated, it does not ensure that
  ///                 their values are not used
  enum ParameterSubset
  {
    None,
    Constant,
    Linear,
    All,
  };

  /// Dummy constructor with no correction on deterministic part and default values for noises (only
  HolonomicWalkModel();

  /// Constructor with specific parameters set
  HolonomicWalkModel(ParameterSubset deterministic_type, ParameterSubset noise_type);

  ParameterSubset getDeterministicType() const;
  ParameterSubset getNoiseType() const;

  /// TODO: set coefficients

  /// Return the value of the parameters according to current deterministic and
  /// noise types
  virtual Eigen::VectorXd getParameters() const override;

  /// Throw a logic_error if size of new_params does not match expected size
  virtual void setParameters(const Eigen::VectorXd& new_params) override;

  virtual std::vector<std::string> getParametersNames() const override;

  /// Throws an exception if input is not of the type WalkOrderSequence
  virtual Eigen::VectorXd predictObservation(const Input& input, std::default_random_engine* engine) const override;

  /// If engine is nullptr, then only the deterministic part is added
  Eigen::VectorXd getStepDisplacement(const Eigen::Vector3d& expected_move, std::default_random_engine* engine) const;

  /// If engine is nullptr, then only the deterministic part is added
  Eigen::VectorXd getNextPosition(const Eigen::Vector3d& initial, const Eigen::Vector3d& expected_move,
                                  std::default_random_engine* engine) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const;

private:
  /// First column is deterministic_bias, following columns are deterministic_prop
  Eigen::MatrixXd deterministic_coeffs;

  /// First column is noise_constant, following columns are noise_prop
  Eigen::MatrixXd noise_coefficients;

  /// What type of parametrization is used for the deterministic part
  ParameterSubset deterministic_type;

  /// What type of parametrization is used for the noise part
  ParameterSubset noise_type;
};

}  // namespace starkit_model_learning

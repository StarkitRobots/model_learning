#pragma once

#include "starkit_utils/serialization/json_serializable.h"

namespace starkit_model_learning
{
class Model;
class ModelPrior;

class ModelSpace : public starkit_utils::JsonSerializable
{
public:
  /// Return the limits of the parameter space for the given model
  /// Each line represent a new dimension, col0 is min, col1 is max
  virtual Eigen::MatrixXd getParametersSpace(const Model& m, const ModelPrior& prior) const = 0;

  /// Return the limits of the parameter space for the used indices of the
  /// provided model. Each line represent a new dimension, col0 is min, col1 is
  /// max
  Eigen::MatrixXd getParametersSpace(const Model& m, const ModelPrior& prior, const std::set<int>& used_indices) const;

  /// Display one line of parameter space with names on each line
  void append(const Model& m, const ModelPrior& prior, const std::set<int>& used_indices, std::ostream& out) const;
};

}  // namespace starkit_model_learning

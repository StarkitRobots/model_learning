#pragma once

#include "rhoban_utils/serialization/json_serializable.h"

namespace rhoban_model_learning
{

class Model;

class ModelSpace : public rhoban_utils::JsonSerializable {
public:
  /// Return the limits of the parameter space for the given model
  /// Each line represent a new dimension, col0 is min, col1 is max
  virtual Eigen::MatrixXd getParametersSpace(const Model & m) const = 0;

  /// Return the limits of the parameter space for the used indices of the
  /// provided model. Each line represent a new dimension, col0 is min, col1 is
  /// max
  Eigen::MatrixXd getParametersSpace(const Model & m,
                                     const std::vector<int> & used_indices) const;

  /// Display one line of parameter space with names on each line
  void append(const Model & m, const std::vector<int> & used_indices,
              std::ostream & out) const;
};

}

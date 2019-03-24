#pragma once

#include "rhoban_utils/serialization/json_serializable.h"

#include <memory>
#include <set>

namespace rhoban_model_learning
{
class Model : public rhoban_utils::JsonSerializable
{
public:
  Model();

  /// Return the total number of parameters of the model
  virtual int getParametersSize() const;

  /// Return the values of the current parameters
  virtual Eigen::VectorXd getParameters() const = 0;
  virtual Eigen::VectorXd getParameters(const std::set<int>& used_indices) const;

  /// Update the internal structure of the model with the provided parameters
  virtual void setParameters(const Eigen::VectorXd& new_params) = 0;
  /// Set parameters at provided indices
  virtual void setParameters(const Eigen::VectorXd& new_params, const std::set<int>& used_indices);

  /// Return a list of names for the parameters
  /// Default implementation return a vector with {param1,param2,...}
  virtual std::vector<std::string> getParametersNames() const;
  /// List of names for parameters at the given indices
  std::vector<std::string> getParametersNames(const std::set<int>& used_indices) const;

  /// Return the list of indices of the parameters from the provided name.
  /// Names can be 'keywords' representing multiple indices. e.g. "all"
  /// represent all available parameters
  ///
  /// throws a std::out_of_range exception if name is not known
  virtual std::set<int> getIndicesFromName(const std::string& name) const;

  /// Return the list of indices of the parameters from the provided names.
  ///
  /// throws a std::out_of_range exception if one of the names is not known
  /// throws a std::runtime_error if an index is duplicated
  std::set<int> getIndicesFromNames(const std::vector<std::string>& names) const;

  /// Default method for cloning is to serialize the object to Json and deserialize
  /// it which might be too time consuming for some classes
  virtual std::unique_ptr<Model> clone() const;
};

}  // namespace rhoban_model_learning

#pragma once

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{

/// The modular model class allows to publish only a subset of the global set of
/// parameters for model trainers
class ModularModel : public Model
{
public:
  ModularModel();
  ModularModel(const ModularModel & other);

  /// Use all dimensions as default
  ModularModel(int nb_dims);

  virtual Eigen::VectorXd getGlobalParameters() const = 0;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const = 0;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) = 0;
  virtual std::vector<std::string> getGlobalParametersNames() const = 0;

  Eigen::VectorXd getParameters() const override;
  Eigen::MatrixXd getParametersSpace() const override;
  void setParameters(const Eigen::VectorXd & new_params) override;
  std::vector<std::string> getParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;

private:
  /// The list of indices used for training
  std::vector<int> used_indices;

};

}

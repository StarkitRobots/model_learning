#pragma once

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{

/// Implements composition of models with respect to parameters handling and
/// serialization
///
/// A few important points:
/// - Constructors of the child class has to register the models in the map
///   - Except copy constructors (just call copy constructors of CompositeModel)
/// - Only 'mandatory' function to implement for the child class is 'predictObservation'
class CompositeModel : public Model
{
public:
  CompositeModel();
  CompositeModel(const CompositeModel & other);

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd & new_params) override;
  std::vector<std::string> getParametersNames() const override;
  
  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  virtual std::unique_ptr<Model> clone() const override;
  
protected:
  // Each model is named to facilitate 
  std::map<std::string,std::unique_ptr<Model>> models;
};

}

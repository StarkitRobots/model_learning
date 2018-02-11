#pragma once

#include "rhoban_model_learning/model.h"


namespace rhoban_model_learning
{

/// This class is used as a preliminary test for working with angular spaces
///
/// This model includes:
/// - observation noise (at start and end)
/// - step noise(after each order)
class SimpleAngularModel : public Model
{

public:
  SimpleAngularModel();

  SimpleAngularModel(double observation_stddev, double step_stddev);

  virtual Eigen::VectorXd getParameters() const override;
  virtual void setParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getParametersNames() const override;

  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const override;

  virtual double computeLogLikelihood(const Sample & sample,
                                      std::default_random_engine * engine) const override;

  virtual std::unique_ptr<Model> clone() const override;


  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

private:
  double observation_stddev;

  double step_stddev;
};

}

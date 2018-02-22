#pragma once

#include "rhoban_model_learning/model.h"

#include "rhoban_bbo/optimizer.h"

namespace rhoban_model_learning
{

/// A class allowing to optimize the parameters of a model using logLikelihood
class ModelLearner : public rhoban_utils::JsonSerializable
{
public:
  struct Result {
    std::unique_ptr<Model> model;
    double training_log_likelihood;
    double validation_log_likelihood;
  };

  ModelLearner();

  ModelLearner(std::unique_ptr<Model> model,
               std::unique_ptr<rhoban_bbo::Optimizer> optimizer,
               const Eigen::MatrixXd & space,
               const Eigen::VectorXd & initial_guess);

  Result learnParameters(const DataSet & data_set,
                         std::default_random_engine * engine);

  Result learnParameters(const SampleVector & training_set,
                         const SampleVector & validation_set,
                         std::default_random_engine * engine);

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;


  const Model & getModel();

protected:
  /// The model which will be learnt
  std::unique_ptr<Model> model;

  /// The blackbox optimizer used for parameters calibration
  std::unique_ptr<rhoban_bbo::Optimizer> optimizer;

  /// The parameter space
  //TODO: move to model ? (could be overrided with learnParameters)
  Eigen::MatrixXd space;

  /// The initial guess for the parameters set
  //TODO: move to model ? (could be overrided with learnParameters)
  Eigen::VectorXd initial_guess;
};

}

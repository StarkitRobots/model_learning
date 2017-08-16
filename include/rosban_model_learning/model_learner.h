#pragma once

#include "rosban_model_learning/model.h"

#include "rosban_bbo/optimizer.h"

namespace rosban_model_learning
{

/// A class allowing to optimize the parameters of a model using logLikelihood
class ModelLearner
{
public:

  struct Result {
    double training_log_likelihood;
    double validation_log_likelihood;
    Eigen::VectorXd best_parameters;
  };

  Result learnParameters(const SampleVector & training_set,
                         const SampleVector & validation_set,
                         std::default_random_engine * engine);

protected:
  /// The model which
  std::unique_ptr<Model> model;

  /// The blackbox optimizer used for parameters calibration
  std::unique_ptr<rosban_bbo::Optimizer> optimizer;

  /// The parameter space
  Eigen::MatrixXd space;

  /// The initial guess for the parameters set
  Eigen::VectorXd initial_guess;
};

}

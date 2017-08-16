#pragma once

#include "rosban_model_learning/model.h"

namespace rosban_model_learning
{

/// A class allowing to optimize the parameters of a model using logLikelihood
class ModelLearner
{
public:
  Eigen::VectorXd learnParameters(const SampleVector & training_set,
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

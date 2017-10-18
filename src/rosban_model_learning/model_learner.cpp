#include "rosban_model_learning/model_learner.h"

namespace rosban_model_learning
{


ModelLearner::ModelLearner(std::unique_ptr<Model> model_,
                           std::unique_ptr<rosban_bbo::Optimizer> optimizer_,
                           const Eigen::MatrixXd & space_,
                           const Eigen::VectorXd & initial_guess_) :
  model(std::move(model_)), optimizer(std::move(optimizer_)),
  space(space_), initial_guess(initial_guess_)
{
}

ModelLearner::Result
ModelLearner::learnParameters(const SampleVector & training_set,
                              const SampleVector & validation_set,
                              std::default_random_engine * engine)
{
  Result result;
  rosban_bbo::Optimizer::RewardFunc reward_function =
    [this, &training_set, &validation_set]
    (const Eigen::VectorXd & parameters, std::default_random_engine * engine)
    {
      // Copy the original model, update the parameters and compute logLikelihood
      std::unique_ptr<Model> model_copy = this->model->clone();
      model_copy->setParameters(parameters);
      return model_copy->averageLogLikelihood(training_set, engine);
    };
  optimizer->setLimits(space);
  result.best_parameters = optimizer->train(reward_function, initial_guess,
                                            engine);
  // Copy the model 
  std::unique_ptr<Model> model_copy = model->clone();
  model_copy->setParameters(result.best_parameters);
  // Estimate log likelihood on both, training set and validation set
  result.training_log_likelihood =
    model_copy->averageLogLikelihood(training_set, engine);
  result.validation_log_likelihood =
    model_copy->averageLogLikelihood(validation_set, engine);
  return result;
}

}

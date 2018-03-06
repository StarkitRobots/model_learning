#include "rhoban_model_learning/model_learner.h"

#include "rhoban_model_learning/model_factory.h"

#include "rhoban_bbo/optimizer_factory.h"

using namespace rhoban_bbo;

namespace rhoban_model_learning
{

ModelLearner::ModelLearner() {}

ModelLearner::ModelLearner(std::unique_ptr<Model> model_,
                           std::unique_ptr<rhoban_bbo::Optimizer> optimizer_,
                           const Eigen::MatrixXd & space_,
                           const Eigen::VectorXd & initial_guess_) :
  model(std::move(model_)), optimizer(std::move(optimizer_)),
  space(space_), initial_guess(initial_guess_)
{
}

ModelLearner::Result
ModelLearner::learnParameters(const DataSet & data,
                              std::default_random_engine * engine)
{
  return learnParameters(data.training_set, data.validation_set, engine);
}

ModelLearner::Result
ModelLearner::learnParameters(const SampleVector & training_set,
                              const SampleVector & validation_set,
                              std::default_random_engine * engine)
{
  Result result;
  rhoban_bbo::Optimizer::RewardFunc reward_function =
    [this, &training_set]
    (const Eigen::VectorXd & parameters, std::default_random_engine * engine)
    {
      // Copy the original model, update the parameters and compute logLikelihood
      std::unique_ptr<Model> model_copy = this->model->clone();
      model_copy->setParameters(parameters);
      return model_copy->averageLogLikelihood(training_set, engine);
    };
  optimizer->setLimits(space);
  Eigen::VectorXd best_parameters; 
  best_parameters = optimizer->train(reward_function, initial_guess, engine);
  // Copy the model 
  result.model = model->clone();
  result.model->setParameters(best_parameters);
  // Estimate log likelihood on both, training set and validation set
  result.training_log_likelihood =
    result.model->averageLogLikelihood(training_set, engine);
  result.validation_log_likelihood =
    result.model->averageLogLikelihood(validation_set, engine);
  return result;
}

std::string ModelLearner::getClassName() const {
  return "ModelLearner";
}

Json::Value ModelLearner::toJson() const
{
  Json::Value v;
  if (model) {
    v["model"] = model->toFactoryJson();
  }
  if (optimizer) {
    v["optimizer"] = optimizer->toFactoryJson();
  }
  v["space"] = rhoban_utils::matrix2Json(space);
  v["initial_guess"] = rhoban_utils::vector2Json(initial_guess);
  return v;
}

void ModelLearner::fromJson(const Json::Value & v, const std::string & dir_name)
{
  model = ModelFactory().read(v, "model", dir_name);
  optimizer = OptimizerFactory().read(v, "optimizer", dir_name);
  space = rhoban_utils::readEigen<-1,-1>(v, "space");
  rhoban_utils::tryReadEigen(v, "initial_guess", &initial_guess);
  if (initial_guess.rows() == 0) {
    initial_guess = (space.col(0) + space.col(1)) / 2;
  }
}

}

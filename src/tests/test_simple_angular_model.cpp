#include "starkit_model_learning/model_learner.h"
#include "starkit_model_learning/models/simple_angular_model.h"
#include "starkit_model_learning/models/simple_angular_model_input.h"

#include "starkit_random/tools.h"

#include "starkit_bbo/cmaes_optimizer.h"

#include <iostream>

using namespace starkit_model_learning;

int main()
{
  /// Parameters
  int nb_learnings = 5;
  std::map<int, int> steps_map = { { 2, 10 }, { 10, 10 }, { 20, 10 }, { 50, 10 }, { 100, 10 } };
  double obs_stddev = 0.002;
  double step_stddev = 0.005;
  double validation_ratio = 0.2;

  SimpleAngularModel training_model(obs_stddev, step_stddev);
  std::default_random_engine engine = starkit_random::getRandomEngine();
  SampleVector samples;

  for (const auto& e : steps_map)
  {
    int nb_steps = e.first;
    int nb_seq = e.second;

    for (int seq = 0; seq < nb_seq; seq++)
    {
      std::unique_ptr<Input> input(new SimpleAngularModelInput(nb_steps));
      Eigen::VectorXd obs = training_model.predictObservation(*input, &engine);
      std::unique_ptr<Sample> s(new Sample(std::move(input), obs));
      samples.push_back(std::move(s));
    }
  }

  // Learning phase

  for (int learning = 0; learning < nb_learnings; learning++)
  {
    // Reset of all entities
    std::unique_ptr<Model> training_model(new SimpleAngularModel());
    std::unique_ptr<starkit_bbo::Optimizer> optimizer(new starkit_bbo::CMAESOptimizer());
    optimizer->setMaxCalls(250);
    Eigen::Vector2d initial_guess(0.001, 0.001);
    ModelLearner learner(std::move(training_model), std::move(optimizer), initial_guess);
    // Separating samples in two sets
    DataSet data = splitSamples(samples, validation_ratio, &engine);
    // Computing learning
    ModelLearner::Result r = learner.learnParameters(data.training_set, data.validation_set, &engine);
    // Showing results
    std::cout << "training score: " << r.training_log_likelihood << std::endl;
    std::cout << "validation score: " << r.validation_log_likelihood << std::endl;
    std::cout << r.model->getParameters().transpose() << std::endl;
  }
}

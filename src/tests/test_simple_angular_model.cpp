#include "rosban_model_learning/model_learner.h"
#include "rosban_model_learning/models/simple_angular_model.h"
#include "rosban_model_learning/models/simple_angular_model_input.h"

#include "rosban_random/tools.h"

#include "rosban_bbo/cmaes_optimizer.h"

using namespace rosban_model_learning;

int main()
{
  /// Parameters
  int nb_learnings = 5;
  std::map<int,int> steps_map = { {10,10},{20,20}};
  double obs_stddev = 0.2;
  double step_stddev = 0.05;

  SimpleAngularModel training_model(obs_stddev, step_stddev);
  std::default_random_engine engine = rosban_random::getRandomEngine();
  SampleVector samples;

  for (const auto & e : steps_map) {
    int nb_steps = e.first;
    int nb_seq = e.second;

    for (int seq = 0; seq < nb_seq; seq++) {
      std::unique_ptr<Input> input(new SimpleAngularModelInput(nb_steps));
      Eigen::VectorXd obs = training_model.predictObservation(*input, &engine);
      std::unique_ptr<Sample> s(new Sample(std::move(input), obs));
      samples.push_back(std::move(s));
    }
  }

  // Learning phase:

  for (int learning = 0; learning < nb_learnings; learning++) {
    std::unique_ptr<Model> training_model(new SimpleAngularModel());
    std::unique_ptr<rosban_bbo::Optimizer> optimizer(new rosban_bbo::CMAESOptimizer());
    Eigen::MatrixXd space(2,2);
    space << 0, 1, 0, 1;
    Eigen::Vector2d initial_guess(0.001, 0.001);
    ModelLearner learner(std::move(training_model), std::move(optimizer),
                         initial_guess);
    ModelLearner::Result r = learner->learnParameters(...,...,&engine);
    std::cout << r.best_parameters.transpose() << std::endl;
  }
}

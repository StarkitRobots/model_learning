#include "rosban_model_learning/model.h"

namespace rosban_model_learning
{

double Model::computeLogLikelihood(const Sample & sample,
                                   const Eigen::VectorXd & parameters,
                                   std::default_random_engine * engine) const
{
  std::vector<Eigen::VectorXd> observations;
  for (int i = 0; i < nb_samples; i++) {
    observations.push_back(predictObservation(sample.getInput(), engine));
  }
  // TODO: fit the distribution and add it
}

double Model::computeLogLikelihood(const SampleVector & data_set,
                                   const Eigen::VectorXd & parameters,
                                   std::default_random_engine * engine) const
{
  double log_likelihood = 0.0;
  for (const Sample & sample : data_set) {
    log_likelihood += computeLogLikelihood(sample, parameters, engine);
  }
  return log_likelihood;
}


}

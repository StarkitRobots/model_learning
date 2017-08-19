#include "rosban_model_learning/model.h"

namespace rosban_model_learning
{

double Model::computeLogLikelihood(const Sample & sample,
                                   std::default_random_engine * engine) const
{
  std::vector<Eigen::VectorXd> observations;
  for (int i = 0; i < nb_samples; i++) {
    observations.push_back(predictObservation(sample.getInput(), engine));
  }
  rosban_random::MultivariateGaussian distrib;
  distrib.fit(observations, getObservationsCircularity()); 
  // TODO: fit the distribution and add it
}

double Model::computeLogLikelihood(const SampleVector & data_set,
                                   std::default_random_engine * engine) const
{
  double log_likelihood = 0.0;
  for (const std::unique_ptr<Sample> & sample : data_set) {
    log_likelihood += computeLogLikelihood(*sample, engine);
  }
  return log_likelihood;
}


}

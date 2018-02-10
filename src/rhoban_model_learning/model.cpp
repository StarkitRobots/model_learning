#include "rhoban_model_learning/model.h"

#include "rhoban_random/multivariate_gaussian.h"

namespace rhoban_model_learning
{

Model::Model() : nb_samples(500)
{
}

/// Number of samples to estimate logLikelihood
Model::Model(int nb_samples_) : nb_samples(nb_samples_)
{
}

double Model::computeLogLikelihood(const Sample & sample,
                                   std::default_random_engine * engine) const
{
  std::vector<Eigen::VectorXd> observations;
  for (int i = 0; i < nb_samples; i++) {
    observations.push_back(predictObservation(sample.getInput(), engine));
  }
  rhoban_random::MultivariateGaussian distrib;
  distrib.fit(observations, getObservationsCircularity()); 
  // TODO: fit the distribution and add it
  return distrib.getLogLikelihood(sample.getObservation());
}

double Model::averageLogLikelihood(const SampleVector & data_set,
                                   std::default_random_engine * engine) const
{
  double log_likelihood = 0.0;
  for (const std::unique_ptr<Sample> & sample : data_set) {
    log_likelihood += computeLogLikelihood(*sample, engine);
  }
  return log_likelihood / data_set.size();
}


}

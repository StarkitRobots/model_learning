#include "rhoban_model_learning/model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_utils/threading/multi_core.h"

using namespace rhoban_utils;

namespace rhoban_model_learning
{

Model::Model() : nb_samples(500), nb_threads(1)
{
}

Model::Model(int nb_samples_) : nb_samples(nb_samples_), nb_threads(1)
{
}

Model::Model(const Model & other)
  : nb_samples(other.nb_samples), nb_threads(other.nb_threads)
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
  // Preparing task
  std::vector<double> log_likelihoods(data_set.size());
  MultiCore::StochasticTask log_likelihood_computer =
    [&](int start_idx, int end_idx, std::default_random_engine * engine)
    {
      for (int idx = start_idx; idx < end_idx; idx++) {
        log_likelihoods[idx] = this->computeLogLikelihood(*(data_set[idx]), engine);
      }
    };
  // Computing evaluation
  MultiCore::runParallelStochasticTask(log_likelihood_computer, data_set.size(),
                                       nb_threads, engine);
  // Gathering Results
  double log_likelihood = 0.0;
  for (size_t idx = 0; idx < data_set.size(); idx++) {
    log_likelihood += log_likelihoods[idx];
  }
  return log_likelihood / data_set.size();
}

Json::Value Model::toJson() const {
  Json::Value v;
  v["nb_samples"] = nb_samples;
  v["nb_threads"] = nb_threads;
  return v;
}

void Model::fromJson(const Json::Value & v, const std::string & dir_name) {
  rhoban_utils::tryRead(v, "nb_samples", &nb_samples);
  rhoban_utils::tryRead(v, "nb_threads", &nb_threads);
}


}

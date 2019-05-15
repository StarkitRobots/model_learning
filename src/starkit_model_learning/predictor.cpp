#include "starkit_model_learning/predictor.h"

#include "starkit_random/multivariate_gaussian.h"
#include "starkit_utils/threading/multi_core.h"

using namespace starkit_utils;

namespace starkit_model_learning
{
Predictor::Predictor() : nb_samples(500), nb_threads(1), output_dim(1)
{
}

Predictor::Predictor(const Predictor& other) : nb_samples(other.nb_samples), nb_threads(other.nb_threads)
{
}

Eigen::VectorXi Predictor::getObservationsCircularity() const
{
  return Eigen::VectorXi(output_dim, 0);
}

double Predictor::computeLogLikelihood(const Sample& sample, const Model& model,
                                       std::default_random_engine* engine) const
{
  std::vector<Eigen::VectorXd> observations;
  for (int i = 0; i < nb_samples; i++)
  {
    observations.push_back(predictObservation(sample.getInput(), model, engine));
  }
  starkit_random::MultivariateGaussian distrib;
  distrib.fit(observations, getObservationsCircularity());
  return distrib.getLogLikelihood(sample.getObservation());
}

double Predictor::averageLogLikelihood(const SampleVector& data_set, const Model& model,
                                       std::default_random_engine* engine) const
{
  // Preparing task
  std::vector<double> log_likelihoods(data_set.size());
  MultiCore::StochasticTask log_likelihood_computer = [&](int start_idx, int end_idx,
                                                          std::default_random_engine* engine) {
    for (int idx = start_idx; idx < end_idx; idx++)
    {
      log_likelihoods[idx] = this->computeLogLikelihood(*(data_set[idx]), model, engine);
    }
  };
  // Computing evaluation
  MultiCore::runParallelStochasticTask(log_likelihood_computer, data_set.size(), nb_threads, engine);
  // Gathering Results
  double log_likelihood = 0.0;
  for (size_t idx = 0; idx < data_set.size(); idx++)
  {
    log_likelihood += log_likelihoods[idx];
  }
  return log_likelihood / data_set.size();
}

Json::Value Predictor::toJson() const
{
  Json::Value v;
  v["nb_samples"] = nb_samples;
  v["nb_threads"] = nb_threads;
  v["output_dim"] = output_dim;
  return v;
}

void Predictor::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryRead(v, "nb_samples", &nb_samples);
  starkit_utils::tryRead(v, "nb_threads", &nb_threads);
  starkit_utils::tryRead(v, "output_dim", &output_dim);
}

}  // namespace starkit_model_learning

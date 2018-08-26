#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/model_factory.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_utils/threading/multi_core.h"
#include "rhoban_utils/util.h"

using namespace rhoban_utils;

namespace rhoban_model_learning
{

Model::Model() : nb_samples(500), nb_threads(1)
{
}

Model::Model(const Model & other)
  : nb_samples(other.nb_samples), nb_threads(other.nb_threads)
{
}

int Model::getParametersSize() const {
  return getParameters().rows();
}

Eigen::VectorXd Model::getParameters(const std::vector<int> & used_indices) const {
  Eigen::VectorXd all_parameters = getParameters();
  Eigen::VectorXd used_parameters(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_parameters(used_idx) = all_parameters[idx];
    used_idx++;
  }
  return used_parameters;
}

void Model::setParameters(const Eigen::VectorXd & new_params,
                          const std::vector<int> & used_indices) {
  Eigen::VectorXd all_parameters = getParameters();
  int used_idx = 0;
  for (int idx : used_indices) {
    all_parameters(idx) = new_params(used_idx);
    used_idx++;
  }
  setParameters(all_parameters);
}

std::vector<std::string> Model::getParametersNames() const {
  int nb_parameters = getParametersSize();
  std::vector<std::string> result;
  for (int idx = 0; idx < nb_parameters; idx++) {
    result.push_back("param" + std::to_string(idx+1));
  }
  return result;
}

std::vector<std::string> Model::getParametersNames(const std::vector<int> & used_indices) const {
  std::vector<std::string> all_names = getParametersNames();
  std::vector<std::string> used_names(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_names[used_idx] = all_names[idx];
    used_idx++;
  }
  return used_names;
}

Eigen::VectorXi Model::getObservationsCircularity() const {
  return Eigen::VectorXi(getParametersSize(), 0);
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
  (void) dir_name;
  rhoban_utils::tryRead(v, "nb_samples", &nb_samples);
  rhoban_utils::tryRead(v, "nb_threads", &nb_threads);
}

std::unique_ptr<Model> Model::clone() const {
  Json::Value v = toJson();
  std::unique_ptr<Model> other = ModelFactory().build(getClassName());
  other->fromJson(v,"./");
  return other;
}


}

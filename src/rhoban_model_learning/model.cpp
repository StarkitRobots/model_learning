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
  : nb_samples(other.nb_samples), nb_threads(other.nb_threads), used_indices(other.used_indices)
{
}

int Model::getGlobalParametersCount() const {
  return getGlobalParameters().rows();
}

Eigen::VectorXd Model::getParameters() const {
  Eigen::VectorXd global_parameters = getGlobalParameters();
  Eigen::VectorXd used_parameters(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_parameters(used_idx) = global_parameters[idx];
    used_idx++;
  }
  return used_parameters;
}

Eigen::MatrixXd Model::getParametersSpace() const {
  Eigen::MatrixXd global_space = getGlobalParametersSpace();
  Eigen::MatrixXd used_space(used_indices.size(), 2);
  int used_idx = 0;
  for (int idx : used_indices) {
    used_space.row(used_idx) = global_space.row(idx);
    used_idx++;
  }
  return used_space;
}

void Model::setParameters(const Eigen::VectorXd & new_params) {
  Eigen::VectorXd global_parameters = getGlobalParameters();
  int used_idx = 0;
  for (int idx : used_indices) {
    global_parameters(idx) = new_params(used_idx);
    used_idx++;
  }
  setGlobalParameters(global_parameters);
}

std::vector<std::string> Model::getParametersNames() const {
  std::vector<std::string> global_names = getGlobalParametersNames();
  std::vector<std::string> used_names(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_names[used_idx] = global_names[idx];
    used_idx++;
  }
  return used_names;
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
  v["used_indices"] = rhoban_utils::vector2Json(used_indices);
  return v;
}

void Model::fromJson(const Json::Value & v, const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryRead(v, "nb_samples", &nb_samples);
  rhoban_utils::tryRead(v, "nb_threads", &nb_threads);
  rhoban_utils::tryReadVector<int>(v, "used_indices", &used_indices);
}

void Model::appendParametersSpace(std::ostream & out) const {
  std::vector<std::string> parameters_names = getParametersNames();
  Eigen::MatrixXd parameters_spaces = getParametersSpace();
  if (parameters_names.size() != (size_t)parameters_spaces.rows()) {
    throw std::logic_error(DEBUG_INFO +
                           " inconsistent number of parameters between spaces and names");
  }
  for (int i = 0; i < parameters_spaces.rows(); i++) {
    out << parameters_names[i] << ": ["
        << parameters_spaces(i,0) << "," << parameters_spaces(i,1) << "]"
        << std::endl;
  }
}

std::unique_ptr<Model> Model::clone() const {
  Json::Value v = toJson();
  std::unique_ptr<Model> other = ModelFactory().build(getClassName());
  other->fromJson(v,"./");
  return other;
}


}

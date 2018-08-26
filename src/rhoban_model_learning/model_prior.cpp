#include "rhoban_model_learning/model_prior.h"

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/tools.h"

#include "rhoban_random/gaussian_distribution.h"

#include "rhoban_utils/util.h"

#include <iostream>

namespace rhoban_model_learning
{

Eigen::VectorXd ModelPrior::getParametersMeans(const Model & m,
                                               const std::vector<int> & used_indices) const
{
  return extractSubset(getParametersMeans(m), used_indices);
}

Eigen::VectorXd ModelPrior::getParametersStdDev(const Model & m,
                                               const std::vector<int> & used_indices) const
{
  return extractSubset(getParametersStdDev(m), used_indices);
}

double ModelPrior::getLogLikelihood(const Model & m) const {
  Eigen::VectorXd means = getParametersMeans(m);
  Eigen::VectorXd deviations = getParametersStdDev(m);
  Eigen::VectorXd parameters = m.getParameters();
  double log_likelihood = 0.0;
  for (int i = 0; i < deviations.rows(); i++) {
    double stddev = deviations(i);
    if (stddev <= 0.0) {
      throw std::logic_error(DEBUG_INFO + "Negative or null stddev found");
    }
    rhoban_random::GaussianDistribution distrib(means(i), stddev * stddev);
    log_likelihood += distrib.getLogLikelihood(parameters(i));
  }
  return log_likelihood;
}

double ModelPrior::getLogLikelihood(const Model & m,
                                    const std::vector<int> & used_indices) const {
  Eigen::VectorXd means = getParametersMeans(m);
  Eigen::VectorXd deviations = getParametersStdDev(m);
  Eigen::VectorXd parameters = m.getParameters();
  double log_likelihood = 0.0;
  for (int i : used_indices) {
    double stddev = deviations(i);
    if (stddev <= 0.0) {
      throw std::logic_error(DEBUG_INFO + "Negative or null stddev found");
    }
    rhoban_random::GaussianDistribution distrib(means(i), stddev * stddev);
    log_likelihood += distrib.getLogLikelihood(parameters(i));
  }
  return log_likelihood;
}

}

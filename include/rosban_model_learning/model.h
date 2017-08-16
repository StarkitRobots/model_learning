#pragma once

#include "rosban_model_learning/sample.h"

namespace rosban_model_learning
{

class Model
{
public:

  /// Update the internal structure of the model with the provided parameters
  virtual void setParameters(const Eigen::VectorXd & new_params) = 0;

  /// Return the predicted observation according to the provided input and
  /// parameters. If 'engine' is null, noise is not considered
  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const = 0;

  /// Compute the logLikelihood of the observation of the sample given
  /// its input
  /// Default method is to predict nb_samples observations with noise and to fit
  /// a multivariate-gaussian on the resulting observations. Then the density is
  /// provided directly using the logLikelihood inside the given distribution
  virtual double computeLogLikelihood(const Sample & sample,
                                      const Eigen::VectorXd & parameters,
                                      std::default_random_engine * engine) const;

  /// Compute the logLikelihood of the given parameters for the specified
  /// dataset
  double computeLogLikelihood(const SampleVector & data_set,
                              const Eigen::VectorXd & parameters,
                              std::default_random_engine * engine) const;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;
};

}
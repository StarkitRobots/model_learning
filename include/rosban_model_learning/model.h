#pragma once

#include "rosban_model_learning/sample.h"

namespace rosban_model_learning
{

class Model
{
public:

  /// Return the values of the current parameters
  virtual Eigen::VectorXd getParameters() const = 0;

  /// Update the internal structure of the model with the provided parameters
  virtual void setParameters(const Eigen::VectorXd & new_params) = 0;

  /// Return a list of names for the parameters
  virtual std::vector<std::string> getParametersNames() const = 0;

  /// Return a vector indicating for each dimension of the observation if it is
  /// a circular dimension in radians
  Eigen::VectorXi getObservationsCircularity() = 0;
  

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
                                      std::default_random_engine * engine) const;

  /// Compute the logLikelihood of the given parameters for the specified
  /// dataset
  double computeLogLikelihood(const SampleVector & data_set,
                              std::default_random_engine * engine) const;

  virtual std::unique_ptr<Model> clone() const = 0;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;
};

}

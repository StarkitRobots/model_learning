#pragma once

#include "rhoban_model_learning/sample.h"

#include "rhoban_utils/serialization/json_serializable.h"

namespace rhoban_model_learning
{

class Model : public rhoban_utils::JsonSerializable
{
public:

  Model();
  /// Number of samples to estimate logLikelihood
  Model(int nb_samples);
  /// Copy constructor
  Model(const Model & other);

  /// Return the values of the current parameters
  virtual Eigen::VectorXd getParameters() const = 0;

  /// Return the space allowed for parameters
  virtual Eigen::MatrixXd getParametersSpace() const = 0;

  /// Update the internal structure of the model with the provided parameters
  virtual void setParameters(const Eigen::VectorXd & new_params) = 0;

  /// Return a list of names for the parameters
  virtual std::vector<std::string> getParametersNames() const = 0;

  /// Return a vector indicating for each dimension of the observation if it is
  /// a circular dimension in radians
  virtual Eigen::VectorXi getObservationsCircularity() const = 0;
  

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

  /// Compute the average logLikelihood of the data set given the current
  /// parameters (relies on computeLogLikelihood)
  double averageLogLikelihood(const SampleVector & data_set,
                              std::default_random_engine * engine) const;

  Json::Value toJson() const;
  void fromJson(const Json::Value & v, const std::string & dir_name);

  virtual std::unique_ptr<Model> clone() const = 0;

  /// Append a readable version of the parameters space to the given stream
  void appendParametersSpace(std::ostream & out) const;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;

  /// The maximal number of threads allowed for computing averageLogLikelihood
  /// of a dataSet
  int nb_threads;
};

}

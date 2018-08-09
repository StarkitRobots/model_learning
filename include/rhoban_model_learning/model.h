#pragma once

#include "rhoban_model_learning/sample.h"

#include "rhoban_utils/serialization/json_serializable.h"

namespace rhoban_model_learning
{

class Model : public rhoban_utils::JsonSerializable
{
public:

  Model();
  /// Copy constructor
  Model(const Model & other);

  /// Return the values of the current parameters
  Eigen::VectorXd getParameters() const;

  /// Return the space allowed for parameters
  Eigen::MatrixXd getParametersSpace() const;

  /// Update the internal structure of the model with the provided parameters
  void setParameters(const Eigen::VectorXd & new_params);

  /// Return a list of names for the parameters
  std::vector<std::string> getParametersNames() const;

  /// Return a vector indicating for each dimension of the observation if it is
  /// a circular dimension in radians
  virtual Eigen::VectorXi getObservationsCircularity() const = 0;

  virtual int getGlobalParametersCount() const;
  virtual Eigen::VectorXd getGlobalParameters() const = 0;
  virtual Eigen::MatrixXd getGlobalParametersSpace() const = 0;
  virtual void setGlobalParameters(const Eigen::VectorXd & new_params) = 0;
  virtual std::vector<std::string> getGlobalParametersNames() const = 0;
  

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

  /// Default method for cloning is to serialize the object to Json and deserialize
  /// it which might be too time consuming for some classes
  virtual std::unique_ptr<Model> clone() const;

  /// Append a readable version of the parameters space to the given stream
  void appendParametersSpace(std::ostream & out) const;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;

  /// The maximal number of threads allowed for computing averageLogLikelihood
  /// of a dataSet
  int nb_threads;

  /// The list of indices used for training
  std::vector<int> used_indices;
};

}

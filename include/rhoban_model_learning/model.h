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
  
  /// Return the total number of parameters of the model
  virtual int getParametersSize() const;

  /// Return the values of the current parameters
  virtual Eigen::VectorXd getParameters() const = 0;
  virtual Eigen::VectorXd getParameters(const std::vector<int> & used_indices) const;

  /// Update the internal structure of the model with the provided parameters
  virtual void setParameters(const Eigen::VectorXd & new_params) = 0;
  /// Set parameters at provided indices
  virtual void setParameters(const Eigen::VectorXd & new_params,
                             const std::vector<int> & used_indices);

  /// Return a list of names for the parameters
  /// Default implementation return a vector with {param1,param2,...}
  virtual std::vector<std::string> getParametersNames() const;
  /// List of names for parameters at the given indices
  std::vector<std::string> getParametersNames(const std::vector<int> & used_indices) const;

  /// Return a vector indicating for each dimension of the observation if it is
  /// a circular dimension in radians
  /// Default implementation is none of the dimension is circular
  virtual Eigen::VectorXi getObservationsCircularity() const;
  

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

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;

  /// Default method for cloning is to serialize the object to Json and deserialize
  /// it which might be too time consuming for some classes
  virtual std::unique_ptr<Model> clone() const;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;

  /// The maximal number of threads allowed for computing averageLogLikelihood
  /// of a dataSet
  int nb_threads;
};

}

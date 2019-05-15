#pragma once

#include "starkit_model_learning/model.h"
#include "starkit_model_learning/sample.h"

#include "starkit_utils/serialization/json_serializable.h"

namespace starkit_model_learning
{
class Predictor : public starkit_utils::JsonSerializable
{
public:
  Predictor();
  Predictor(const Predictor& other);

  /// Return a vector indicating for each dimension of the observation if it is
  /// a circular dimension
  /// Default implementation is none of the dimension is circular
  virtual Eigen::VectorXi getObservationsCircularity() const;

  /// Return the predicted observation according to the provided input and
  /// parameters and model. If 'engine' is null, noise is not considered
  virtual Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                             std::default_random_engine* engine) const = 0;

  /// Compute the logLikelihood of the observation of the sample given
  /// its input
  /// Default method is to predict nb_samples observations with noise and to fit
  /// a multivariate-gaussian on the resulting observations. Then the density is
  /// provided directly using the logLikelihood inside the given distribution
  virtual double computeLogLikelihood(const Sample& sample, const Model& model,
                                      std::default_random_engine* engine) const;

  /// Compute the average logLikelihood of the data set given the current
  /// parameters (relies on computeLogLikelihood)
  double averageLogLikelihood(const SampleVector& data_set, const Model& model,
                              std::default_random_engine* engine) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

protected:
  /// The number of samples used to fit a distribution if necessary
  int nb_samples;

  /// The maximal number of threads allowed for computing averageLogLikelihood
  /// of a dataSet
  int nb_threads;

  /// Number of dimensions on the output
  int output_dim;
};

}  // namespace starkit_model_learning

#pragma once

#include "starkit_model_learning/input.h"

#include <Eigen/Core>

#include <memory>
#include <random>
#include <vector>

namespace starkit_model_learning
{
/// A pure interface which should be implemented by all the 'Sample' class
class Sample
{
public:
  Sample(std::unique_ptr<Input> input, const Eigen::VectorXd& observation);

  virtual const Input& getInput() const;
  const Eigen::VectorXd& getObservation() const;

  virtual std::unique_ptr<Sample> clone() const;

protected:
  std::unique_ptr<Input> input;
  Eigen::VectorXd observation;
};

/// Since samples are generic, we need to have generic collections
typedef std::vector<std::unique_ptr<Sample>> SampleVector;

struct DataSet
{
  SampleVector training_set;
  SampleVector validation_set;
};

DataSet splitSamples(const SampleVector& samples, double validation_ratio, std::default_random_engine* engine);
DataSet splitSamples(const SampleVector& samples, int nb_validation_samples, std::default_random_engine* engine);

}  // namespace starkit_model_learning

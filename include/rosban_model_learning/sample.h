#pragma once

#include "rosban_model_learning/input.h"

#include <Eigen/Core>

#include <memory>

namespace rosban_model_learning
{

/// A pure interface which should be implemented by all the 'Sample' class
class Sample
{
public:

  virtual const Input & getInput() const = 0;
  const Eigen::VectorXd & getObservation() const;

  virtual std::unique_ptr<Sample> clone() const = 0;

protected:
  Eigen::VectorXd observation;
};

/// Since samples are generic, we need to have generic collections
typedef std::vector<std::unique_ptr<Sample>> SampleVector;

}

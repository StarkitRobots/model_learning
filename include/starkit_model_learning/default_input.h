#pragma once

#include "starkit_model_learning/input.h"

#include <Eigen/Core>

namespace starkit_model_learning
{
/// Default inputs are simply a vector of data
class DefaultInput : public Input
{
public:
  DefaultInput();
  DefaultInput(const Eigen::VectorXd& data);
  DefaultInput(const DefaultInput& other);

  std::unique_ptr<Input> clone() const override;

  Eigen::VectorXd data;
};

}  // namespace starkit_model_learning

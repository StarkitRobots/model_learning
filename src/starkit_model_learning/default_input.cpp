#include "starkit_model_learning/default_input.h"

namespace starkit_model_learning
{
DefaultInput::DefaultInput()
{
}

DefaultInput::DefaultInput(const Eigen::VectorXd& data_) : data(data_)
{
}

DefaultInput::DefaultInput(const DefaultInput& other) : data(other.data)
{
}

std::unique_ptr<Input> DefaultInput::clone() const
{
  return std::unique_ptr<Input>(new DefaultInput(*this));
}

}  // namespace starkit_model_learning

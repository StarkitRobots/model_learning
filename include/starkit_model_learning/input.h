#pragma once

#include <memory>

namespace starkit_model_learning
{
/// Simple interface for inputs (no content yet)
class Input
{
public:
  Input(){};
  virtual ~Input(){};

  virtual std::unique_ptr<Input> clone() const = 0;
};

}  // namespace starkit_model_learning

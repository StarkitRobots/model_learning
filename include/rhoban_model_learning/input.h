#pragma once

#include <memory>

namespace rhoban_model_learning
{
/// Simple interface for inputs (no content yet)
class Input
{
public:
  Input(){};
  virtual ~Input(){};

  virtual std::unique_ptr<Input> clone() const = 0;
};

}  // namespace rhoban_model_learning

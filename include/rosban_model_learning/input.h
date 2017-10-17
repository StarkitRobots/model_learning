#pragma once

#include <memory>

namespace rosban_model_learning
{

/// Simple interface for inputs (no content yet)
class Input {
public:
  Input(){};
  virtual ~Input(){};

  virtual std::unique_ptr<Input> clone() const {
    return std::unique_ptr<Input>(new Input());
  }
};

}

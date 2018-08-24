#include "rhoban_model_learning/default_input.h"

namespace rhoban_model_learning {

DefaultInput::DefaultInput() {}
DefaultInput::DefaultInput(const DefaultInput & other) : data(other.data) {}

std::unique_ptr<Input> DefaultInput::clone() const {
  return std::unique_ptr<Input>(new DefaultInput(*this));
}

}

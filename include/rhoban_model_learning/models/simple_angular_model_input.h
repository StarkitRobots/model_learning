#include "rhoban_model_learning/input.h"

namespace rhoban_model_learning
{
class SimpleAngularModelInput : public Input
{
public:
  SimpleAngularModelInput(int nb_steps_) : nb_steps(nb_steps_){};

  virtual std::unique_ptr<Input> clone() const override
  {
    return std::unique_ptr<Input>(new SimpleAngularModelInput(nb_steps));
  }

  int nb_steps;
};

}  // namespace rhoban_model_learning

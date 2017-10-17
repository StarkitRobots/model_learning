#include "rosban_model_learning/input.h"

namespace rosban_model_learning
{

class SimpleAngularModelInput : public Input
{
public:
  SimpleAngularModelInput(int nb_steps_) : nb_steps(nb_steps_) {};

  int nb_steps;
};

}

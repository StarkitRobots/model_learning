#pragma once

#include "rhoban_model_learning/model.h"

#include <rhoban_utils/serialization/factory.h> 

namespace rhoban_model_learning
{

class ModelFactory : public rhoban_utils::Factory<Model> {
public:
  ModelFactory();
};

}

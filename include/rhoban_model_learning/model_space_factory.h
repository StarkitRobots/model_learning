#pragma once

#include "rhoban_model_learning/model_space.h"

#include <rhoban_utils/serialization/factory.h> 

namespace rhoban_model_learning
{

class ModelSpaceFactory : public rhoban_utils::Factory<ModelSpace> {
public:
  ModelSpaceFactory();
};

}

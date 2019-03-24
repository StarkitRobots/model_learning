#pragma once

#include "rhoban_model_learning/model_prior.h"

#include <rhoban_utils/serialization/factory.h>

namespace rhoban_model_learning
{
class ModelPriorFactory : public rhoban_utils::Factory<ModelPrior>
{
public:
  ModelPriorFactory();
};

}  // namespace rhoban_model_learning

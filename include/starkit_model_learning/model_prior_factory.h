#pragma once

#include "starkit_model_learning/model_prior.h"

#include <starkit_utils/serialization/factory.h>

namespace starkit_model_learning
{
class ModelPriorFactory : public starkit_utils::Factory<ModelPrior>
{
public:
  ModelPriorFactory();
};

}  // namespace starkit_model_learning

#pragma once

#include "starkit_model_learning/model_space.h"

#include <starkit_utils/serialization/factory.h>

namespace starkit_model_learning
{
class ModelSpaceFactory : public starkit_utils::Factory<ModelSpace>
{
public:
  ModelSpaceFactory();
};

}  // namespace starkit_model_learning

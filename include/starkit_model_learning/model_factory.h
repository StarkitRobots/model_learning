#pragma once

#include "starkit_model_learning/model.h"

#include <starkit_utils/serialization/factory.h>

namespace starkit_model_learning
{
class ModelFactory : public starkit_utils::Factory<Model>
{
public:
  ModelFactory();
};

}  // namespace starkit_model_learning

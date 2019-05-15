#pragma once

#include "starkit_model_learning/predictor.h"

#include <starkit_utils/serialization/factory.h>

namespace starkit_model_learning
{
class PredictorFactory : public starkit_utils::Factory<Predictor>
{
public:
  PredictorFactory();
};

}  // namespace starkit_model_learning

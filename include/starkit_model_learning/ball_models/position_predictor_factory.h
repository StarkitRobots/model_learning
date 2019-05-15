#pragma once

#include "starkit_model_learning/ball_models/position_predictor.h"

#include "starkit_utils/serialization/factory.h"

namespace starkit_model_learning
{
class PositionPredictorFactory : public starkit_utils::Factory<PositionPredictor>
{
public:
  PositionPredictorFactory();
};

}  // namespace starkit_model_learning

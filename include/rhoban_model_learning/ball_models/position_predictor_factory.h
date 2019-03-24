#pragma once

#include "rhoban_model_learning/ball_models/position_predictor.h"

#include "rhoban_utils/serialization/factory.h"

namespace rhoban_model_learning
{
class PositionPredictorFactory : public rhoban_utils::Factory<PositionPredictor>
{
public:
  PositionPredictorFactory();
};

}  // namespace rhoban_model_learning

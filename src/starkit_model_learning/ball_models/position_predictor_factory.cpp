#include "starkit_model_learning/ball_models/position_predictor_factory.h"
#include "starkit_model_learning/ball_models/ball_physical_model.h"

namespace starkit_model_learning
{
PositionPredictorFactory::PositionPredictorFactory()
{
  registerBuilder("BallPhysicalModel", []() { return std::unique_ptr<PositionPredictor>(new BallPhysicalModel); });
}

}  // namespace starkit_model_learning

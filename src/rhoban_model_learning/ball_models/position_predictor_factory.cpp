#include "rhoban_model_learning/ball_models/position_predictor_factory.h"
#include "rhoban_model_learning/ball_models/ball_physical_model.h"

namespace rhoban_model_learning
{

PositionPredictorFactory::PositionPredictorFactory() {
  registerBuilder("BallPhysicalModel",
                  [](){return std::unique_ptr<PositionPredictor>(new BallPhysicalModel);});  
}

}

#include "rhoban_model_learning/model_factory.h"

#include "rhoban_model_learning/basic_models/linear_model.h"

// TODO: integrate back 
//#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"
//#include "rhoban_model_learning/ball_models/position_predictor_factory.h"
//#include "rhoban_model_learning/ball_models/speed_estimator_factory.h"
//#include "rhoban_model_learning/ball_models/trajectory_predictor.h"


namespace rhoban_model_learning
{

ModelFactory::ModelFactory() {
  registerBuilder("LinearModel", []() { return std::unique_ptr<Model>(new LinearModel); });
//  registerBuilder("VisionCorrectionModel",
//                  []() { return std::unique_ptr<Model>(new VisionCorrectionModel); });
//  registerBuilder("TrajectoryPredictor",
//                  []() { return std::unique_ptr<Model>(new TrajectoryPredictor); });
//  importJsonBuilders(PositionPredictorFactory());
//  importJsonBuilders(SpeedEstimatorFactory());
}

}

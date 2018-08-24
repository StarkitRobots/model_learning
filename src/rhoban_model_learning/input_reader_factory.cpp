#include "rhoban_model_learning/input_reader_factory.h"

// TODO: integrate back readers

//#include "rhoban_model_learning/ball_models/trajectory_predictor.h"
//#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

namespace rhoban_model_learning
{

typedef std::unique_ptr<InputReader> PTR;

InputReaderFactory::InputReaderFactory() {
//  registerBuilder("TrajectoryInputReader",
//                  []() { return PTR(new TrajectoryPredictor::Reader); });
//  registerBuilder("VisionInputReader",
//                  []() { return PTR(new VisionCorrectionModel::VisionInputReader); });
}

}

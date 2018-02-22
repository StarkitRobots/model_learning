#include "rhoban_model_learning/input_reader_factory.h"

#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

namespace rhoban_model_learning
{

typedef std::unique_ptr<InputReader> PTR;

InputReaderFactory::InputReaderFactory() {
  registerBuilder("VisionInputReader",
                  []() { return PTR(new VisionCorrectionModel::VisionInputReader); });
}

}

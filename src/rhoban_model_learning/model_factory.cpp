#include "rhoban_model_learning/model_factory.h"

#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

namespace rhoban_model_learning
{

ModelFactory::ModelFactory() {
  registerBuilder("VisionCorrectionModel",
                  []() { return std::unique_ptr<Model>(new VisionCorrectionModel); });
}

}

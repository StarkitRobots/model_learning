#include "starkit_model_learning/predictor_factory.h"

#include "starkit_model_learning/basic_models/linear_predictor.h"
#include "starkit_model_learning/humanoid_models/poses_optimization_predictor.h"

namespace starkit_model_learning
{
PredictorFactory::PredictorFactory()
{
  registerBuilder("LinearPredictor", []() { return std::unique_ptr<Predictor>(new LinearPredictor); });
  registerBuilder("PosesOptimizationPredictor",
                  []() { return std::unique_ptr<Predictor>(new PosesOptimizationPredictor); });
}

}  // namespace starkit_model_learning

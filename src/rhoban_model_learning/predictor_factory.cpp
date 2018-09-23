#include "rhoban_model_learning/predictor_factory.h"

#include "rhoban_model_learning/basic_models/linear_predictor.h"

namespace rhoban_model_learning
{

PredictorFactory::PredictorFactory() {
  registerBuilder("LinearPredictor", []() { return std::unique_ptr<Predictor>(new LinearPredictor); });
}

}

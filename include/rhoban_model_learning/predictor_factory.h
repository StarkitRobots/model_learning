#pragma once

#include "rhoban_model_learning/predictor.h"

#include <rhoban_utils/serialization/factory.h> 

namespace rhoban_model_learning
{

class PredictorFactory : public rhoban_utils::Factory<Predictor> {
public:
  PredictorFactory();
};

}

#pragma once

#include "rhoban_model_learning/ball_models/speed_estimator.h"

#include "rhoban_utils/serialization/factory.h"

namespace rhoban_model_learning {

class SpeedEstimatorFactory : public rhoban_utils::Factory<SpeedEstimator> {
public:
  SpeedEstimatorFactory();
};

}

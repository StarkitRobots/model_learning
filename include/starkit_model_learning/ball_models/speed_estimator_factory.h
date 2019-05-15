#pragma once

#include "starkit_model_learning/ball_models/speed_estimator.h"

#include "starkit_utils/serialization/factory.h"

namespace starkit_model_learning
{
class SpeedEstimatorFactory : public starkit_utils::Factory<SpeedEstimator>
{
public:
  SpeedEstimatorFactory();
};

}  // namespace starkit_model_learning

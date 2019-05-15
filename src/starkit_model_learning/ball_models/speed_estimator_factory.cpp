#include "starkit_model_learning/ball_models/speed_estimator_factory.h"
#include "starkit_model_learning/ball_models/linear_speed_estimator.h"

namespace starkit_model_learning
{
SpeedEstimatorFactory::SpeedEstimatorFactory()
{
  registerBuilder("LinearSpeedEstimator", []() { return std::unique_ptr<SpeedEstimator>(new LinearSpeedEstimator); });
}

}  // namespace starkit_model_learning

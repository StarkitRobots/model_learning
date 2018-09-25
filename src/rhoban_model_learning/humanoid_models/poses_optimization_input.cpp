#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"

namespace rhoban_model_learning
{

PosesOptimizationInput::PosesOptimizationInput() : image_id(-1), aruco_id(-1)
{
}

PosesOptimizationInput::PosesOptimizationInput(const PosesOptimizationInput & other)
  : image_id(other.image_id), aruco_id(other.aruco_id)
{
}

std::unique_ptr<Input> PosesOptimizationInput::clone() const {
  return std::unique_ptr<Input>(new PosesOptimizationInput(*this));
}

}

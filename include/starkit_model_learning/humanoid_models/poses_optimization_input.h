#pragma once

#include "starkit_model_learning/input.h"

namespace starkit_model_learning
{
class PosesOptimizationInput : public Input
{
public:
  PosesOptimizationInput();
  PosesOptimizationInput(int image_id, int aruco_id);
  PosesOptimizationInput(const PosesOptimizationInput& other);

  std::unique_ptr<Input> clone() const override;

  /// Image identifier
  int image_id;

  /// Marker identifier
  int aruco_id;
};

}  // namespace starkit_model_learning

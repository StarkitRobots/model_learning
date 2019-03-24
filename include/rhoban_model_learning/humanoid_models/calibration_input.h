#pragma once

#include "rhoban_model_learning/input.h"

#include "Types/VectorLabel.hpp"

namespace rhoban_model_learning
{
class CalibrationInput : public Input
{
public:
  CalibrationInput();
  CalibrationInput(const Leph::VectorLabel& data);
  CalibrationInput(const CalibrationInput& other);

  virtual std::unique_ptr<Input> clone() const override;

  /// Contains all DOF + IMU + pixel position
  Leph::VectorLabel data;
};

}  // namespace rhoban_model_learning

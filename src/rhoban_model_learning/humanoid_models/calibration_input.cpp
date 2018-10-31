#include "rhoban_model_learning/humanoid_models/calibration_input.h"

namespace rhoban_model_learning
{

CalibrationInput::CalibrationInput() {}
CalibrationInput::CalibrationInput(const Leph::VectorLabel & data_)
  : data(data_)
{}
CalibrationInput::CalibrationInput(const CalibrationInput & other)
  : data(other.data)
{}


std::unique_ptr<Input> CalibrationInput::clone() const {
  return std::unique_ptr<Input>(new CalibrationInput(*this));
}

}

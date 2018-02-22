#pragma once

#include "rhoban_model_learning/input_reader.h"

#include <rhoban_utils/serialization/factory.h> 

namespace rhoban_model_learning
{

class InputReaderFactory : public rhoban_utils::Factory<InputReader> {
public:
  InputReaderFactory();
};

}

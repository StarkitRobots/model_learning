#pragma once

#include "rhoban_model_learning/data_set_reader.h"

#include <rhoban_utils/serialization/factory.h> 

namespace rhoban_model_learning
{

class DataSetReaderFactory : public rhoban_utils::Factory<DataSetReader> {
public:
  DataSetReaderFactory();
};

}

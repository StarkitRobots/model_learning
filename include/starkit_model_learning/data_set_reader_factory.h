#pragma once

#include "starkit_model_learning/data_set_reader.h"

#include <starkit_utils/serialization/factory.h>

namespace starkit_model_learning
{
class DataSetReaderFactory : public starkit_utils::Factory<DataSetReader>
{
public:
  DataSetReaderFactory();
};

}  // namespace starkit_model_learning

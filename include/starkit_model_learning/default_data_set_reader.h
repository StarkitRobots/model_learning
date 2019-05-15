#pragma once

#include "starkit_model_learning/data_set_reader.h"

namespace starkit_model_learning
{
/// Read data from a csv file, simply consider the first 'input_dim' columns as input
/// and the 'observation_dim' columns as observations
///
/// A percentage of the entries is used as validation set
class DefaultDataSetReader : public DataSetReader
{
public:
  DefaultDataSetReader();

  DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

  std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// Dimensionality of input
  int input_dim;
  /// Dimensionality of output
  int obs_dim;
  /// Ratio of entries inside validation set
  double validation_ratio;
};

}  // namespace starkit_model_learning
